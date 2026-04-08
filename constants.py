class FuelConstants:
    _feeder_id = 10
    _intake_id = 11
    _left_launcher_id = 13
    _right_launcher_id = 14
    _feeder_current_limit = 60
    _intake_current_limit = 60
    _feeder_intake_speed = 0.8
    _intake_intake_speed = 0.4
    _feeder_launch_speed = 1.0
    _intake_launch_speed = 0.5
    _launcher_launch_speed = 1.0
    _feeder_spinup_speed = 0

    _launcher_target_rpm = 4600 # example; set to tuned RPM
    _launcher_kP = 0.00035
    _launcher_kI = 0.0
    _launcher_kD = 0.0
    _launcher_ff = 0.002
    _launcher_rpm_tolerance = 150 # RPM tolerance for "ready"
    _launcher_feed_when_ready = True # boolean if you want feeders gated by readiness

class OperatorConstants:
    _driver_controller = 0
    _operator_controller = 1

class ElevatorConstants:
    _elevator_id = 12
    _elevator_speed = 0.6
