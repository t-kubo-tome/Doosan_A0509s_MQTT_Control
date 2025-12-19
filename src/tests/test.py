import sys
from pathlib import Path

src_path = Path(__file__).parent.parent
sys.path.insert(0, str(src_path))

from doosan.doosan_robot import DoosanRobot


if __name__ == "__main__":
    """
    >> python test.py
    open_connection
    MONITORING_ACCESS_CONTROL_LOSS
    OnTpInitializingCompleted
    STATE_SAFE_OFF
    setup_monitoring_version
    robot.start()=True
    set_robot_control
    MONITORING_ACCESS_CONTROL_GRANT
    STATE_SAFE_OFF
    STATE_SAFE_OFF CONTROL_SERVO_ON
    STATE_SAFE_OFF
    STATE_SAFE_OFF CONTROL_SERVO_ON
    STATE_STANDBY
    robot.enable()=True
    robot.get_current_pose()=[-448.2719421386719, -0.034245822578668594, 417.2590026855469, 0.1496591717004776, -175.78790283203125, 0.14714868366718292]
    robot.get_default_pose()=[-550.0, -50.0, 400.0, 5.0, -135.0, -5.0]
    robot.get_current_joint()=[0.001289734267629683, -10.170038223266602, -82.04082489013672, 0.010966389440000057, -83.5770492553711, -0.0020467836875468493]
    robot.disable()=True
    robot.stop()=True
    """
    log_mode = "queue"  # "none", "cpp", "queue"
    robot = DoosanRobot("192.168.5.43", log_mode)
    # Python側でログを受け取る
    if log_mode == "queue":
        import threading

        def print_log_queue():
            while True:
                log_block = robot.pop_log_queue()
                for log in log_block:
                    print(log)

        log_thread = threading.Thread(target=print_log_queue, daemon=True)
        log_thread.start()

    print(f"{robot.start()=}")
    print(f"{robot.enable()=}")
    print(f"{robot.get_robot_state()=}")
    print(f"{robot.get_default_pose()=}")
    print(f"{robot.get_current_pose()=}")
    print(f"{robot.get_current_joint()=}")
    print(f"{robot.move_default_pose_until_completion()=}")
    print(f"{robot.get_current_pose()=}")
    print(f"{robot.get_current_joint()=}")
    print(f"{robot.disable()=}")
    print(f"{robot.stop()=}")
        