class FuelConstants:
    _feeder_id = 10
    _launcher_id = 11
    _feeder_current_limit = 60
    _launcher_current_limit = 60
    _feeder_voltage = 9.0
    _launcher_voltage = 9.0
    _feeder_intake_speed = 0.5
    _launcher_intake_speed = 0.3
    _feeder_launch_speed = 0.3
    _launcher_launch_speed = 0.8
    _feeder_spinup_speed = 0
    
    # --- Fuel / launcher tuning & vision mapping (place inside FuelConstants) ---
    # Launcher speeds and RPM targets (units: RPM)
    _launcher_launch_speed = 3000.0       # default launch RPM (used if vision not available)
    _launcher_spinup_rpm = 2500.0         # spin-up target RPM
    _launcher_min_rpm = 1000.0
    _launcher_max_rpm = 6000.0

    # Robot-side PID for launcher fallback (units: RPM)
    _launcher_kP = 0.0015
    _launcher_kI = 0.0
    _launcher_kD = 0.0002

    # Simple feedforward gains (units: volts / (revs/sec) etc. — tune to your system)
    # Note: SimpleMotorFeedforward expects units in volts for ks, kv (rev/sec), ka (rev/sec^2).
    _launcher_ff_kS = 0.0
    _launcher_ff_kV = 0.00015
    _launcher_ff_kA = 0.0

    # Vision -> RPM mapping: target physical height (meters) and mapping range
    # Example: if shooting at a target hub with center at ~2.64 m, set that here.
    _target_height_m = 2.64            # replace with real target height in meters
    _target_min_dist_m = 0.5
    _target_max_dist_m = 6.0

    # Optional distance->RPM mapping bounds (used by the placeholder mapping)
    _launcher_min_rpm = _launcher_min_rpm
    _launcher_max_rpm = _launcher_max_rpm

    # Feeder/launcher open-loop speeds (existing values in your repo may already define these)
    # _feeder_launch_speed, _feeder_spinup_speed, _feeder_intake_speed, _launcher_intake_speed
    # (If missing, set them here; units are percent [-1..1] for set() calls)

class OperatorConstants:
    _driver_controller = 0
    _operator_controller = 1

class ElevatorConstants:
    _elevator_id = 12
    _elevator_speed = 0.6
