from typing import Optional, Sequence, List
from networktables import NetworkTables
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import SubsystemBase
import math

# subsystems/vision.py
# Vision subsystem for Limelight AprilTag tracking and pose helpers.
# Designed for RobotPy / command-based usage (commands2).
#
# Provides safe accessors for common Limelight NetworkTables keys
# and a few convenience helpers for alignment and simple distance
# estimation. This file avoids hard assumptions about Limelight NT
# keys and will gracefully return None when data is unavailable.




class Vision(SubsystemBase):
    """
    Vision subsystem to interface with a Limelight camera for AprilTag
    tracking and simple pose/target helpers.

    Typical usage:
      vision = Vision(table_name="limelight")
      if vision.has_target():
          tx = vision.get_tx()
          correction = vision.drive_correction(kP=0.02)
    """

    def __init__(self, table_name: str = "limelight") -> None:
        super().__init__()
        # Get networktables entry for the limelight.
        # Expect the robot application to have NetworkTables initialized.
        self._table = NetworkTables.getTable(table_name)

    # --- Basic limelight accessors ------------------------------------------------

    def has_target(self) -> bool:
        """Return True if limelight reports a valid target (tv == 1)."""
        tv = self._get_number("tv")
        return bool(tv and int(tv) == 1)

    def get_tx(self) -> Optional[float]:
        """Horizontal offset from crosshair to target (degrees)."""
        return self._get_number("tx")

    def get_ty(self) -> Optional[float]:
        """Vertical offset from crosshair to target (degrees)."""
        return self._get_number("ty")

    def get_ta(self) -> Optional[float]:
        """Target area (0-100%)."""
        return self._get_number("ta")

    def get_tid(self) -> Optional[int]:
        """
        Target ID (AprilTag ID). Limelight stores this in 'tid' or 'tID'
        depending on firmware/version. Return int or None.
        """
        val = self._get_number("tid")
        if val is None:
            val = self._get_number("tID")
        if val is None:
            return None
        try:
            return int(val)
        except (ValueError, TypeError):
            return None

    def get_pipeline(self) -> Optional[int]:
        """Current pipeline index (if available)."""
        val = self._get_number("getpipe")
        if val is None:
            return None
        try:
            return int(val)
        except (ValueError, TypeError):
            return None

    def set_pipeline(self, index: int) -> None:
        """Set the limelight pipeline index."""
        self._table.putNumber("pipeline", int(index))

    def set_led(self, on: bool) -> None:
        """
        Control limelight LEDs.
        on=True -> LEDs on (mode 0), False -> force off (mode 1).
        See Limelight docs for LED mode semantics.
        """
        mode = 0 if on else 1
        self._table.putNumber("ledMode", int(mode))

    # --- Pose & raw arrays --------------------------------------------------------

    def get_camtran(self) -> Optional[List[float]]:
        """
        Return the 'camtran' camera translation array if present.
        Format depends on Limelight firmware; may be 6- or 7-element array.
        Return None if not available.
        """
        arr = self._get_array("camtran")
        if arr:
            return list(arr)
        # try alternative keys
        arr = self._get_array("targetpose_cameraspace")
        if arr:
            return list(arr)
        return None

    def get_botpose(self) -> Optional[List[float]]:
        """
        Return a raw botpose array (if available) such as 'botpose_wpiblue'
        or 'botpose_wpired' depending on team alliance. The array contents
        and order depend on Limelight version. This function returns the
        raw list for callers to interpret.
        """
        for key in ("botpose_wpiblue", "botpose_wpired", "botpose"):
            arr = self._get_array(key)
            if arr:
                return list(arr)
        return None

    def get_robot_pose2d(self) -> Optional[Pose2d]:
        """
        Attempt to construct a Pose2d (x, y, yaw) from available botpose
        arrays. This makes assumptions many Limelight users rely on:
          - botpose arrays typically contain x (meters), y (meters),
            z (meters), yaw/pitch/roll (degrees) or similar ordering.
        If a reasonable interpretation cannot be made, return None.
        """
        arr = self.get_botpose()
        if not arr:
            return None

        # Heuristic attempts:
        # - If arr has 6+ elements: assume [x, y, z, yaw, pitch, roll] (degrees)
        # - If arr has 7+ elements: sometimes quaternion order appears; try first 3 + a yaw at index 3
        if len(arr) >= 4:
            try:
                x = float(arr[0])
                y = float(arr[1])
                # try yaw at index 3 if present, else index 5 or 6
                yaw_deg = None
                if len(arr) >= 4:
                    yaw_deg = float(arr[3])
                if yaw_deg is None and len(arr) >= 6:
                    yaw_deg = float(arr[5])
                if yaw_deg is None and len(arr) >= 7:
                    yaw_deg = float(arr[6])
                if yaw_deg is None:
                    return None
                yaw_rad = math.radians(yaw_deg)
                return Pose2d(x, y, Rotation2d(yaw_rad))
            except (ValueError, TypeError):
                return None
        return None

    # --- Convenience helpers ------------------------------------------------------

    def estimate_distance_to_target(self,
                                    camera_height_m: Optional[float] = None,
                                    target_height_m: Optional[float] = None,
                                    camera_pitch_deg: float = 0.0) -> Optional[float]:
        """
        Estimate straight-line distance from camera to a target using vertical
        angle (ty). If camera or target heights are not supplied, returns None.

        Parameters:
          camera_height_m: height of the camera above floor in meters
          target_height_m: height of the target above floor in meters
          camera_pitch_deg: mounting pitch of the camera in degrees (positive up)

        Formula:
          distance = (target_height - camera_height) / tan( camera_pitch + ty )

        Returns:
          distance in meters or None if inputs not available.
        """
        ty = self.get_ty()
        if ty is None or camera_height_m is None or target_height_m is None:
            return None
        total_angle_deg = camera_pitch_deg + ty
        # avoid tan(90deg)
        if abs((total_angle_deg % 180) - 90) < 1e-3:
            return None
        try:
            distance = (target_height_m - camera_height_m) / math.tan(math.radians(total_angle_deg))
            return float(abs(distance))
        except (ZeroDivisionError, ValueError):
            return None

    def drive_correction(self, kP: float = 0.02, deadband_deg: float = 1.0) -> Optional[float]:
        """
        Simple proportional controller output (rotation) to align robot to target.
        Uses tx (horizontal offset). Caller should apply this as a rotation setpoint
        (e.g. turn = clamp(vision.drive_correction()) ).

        Parameters:
          kP: proportional gain
          deadband_deg: if abs(tx) < deadband, return 0.0

        Returns:
          rotation correction (signed) or None if no target.
        """
        tx = self.get_tx()
        if tx is None:
            return None
        if abs(tx) <= deadband_deg:
            return 0.0
        # negative sign because positive tx means target is right of crosshair;
        # many drive controllers use positive rotation to turn robot left -> adjust as desired.
        return -kP * float(tx)

    def alignment_info(self) -> dict:
        """
        Return a compact dict of common alignment info. Useful for logging/dashboard.
        Keys:
          - has_target (bool)
          - tx, ty, ta (floats or None)
          - tid (int or None)
          - pipeline (int or None)
        """
        return {
            "has_target": self.has_target(),
            "tx": self.get_tx(),
            "ty": self.get_ty(),
            "ta": self.get_ta(),
            "tid": self.get_tid(),
            "pipeline": self.get_pipeline(),
        }

    # --- Internal helpers --------------------------------------------------------

    def _get_number(self, key: str) -> Optional[float]:
        """Return a numeric entry or None if not present."""
        try:
            v = self._table.getNumber(key, None)
            # NetworkTables may return None or float; normalize
            if v is None:
                return None
            return float(v)
        except Exception:
            return None

    def _get_array(self, key: str) -> Optional[Sequence]:
        """Return an array entry or None if not present."""
        try:
            arr = self._table.getNumberArray(key, [])
            if arr is None or len(arr) == 0:
                return None
            return arr
        except Exception:
            return None