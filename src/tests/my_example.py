import psutil
import os
import sys
import time
import threading

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO
from dashboard_client import DashboardClient

from utils import (
    deg2rad,
    deg2rad_list,
    rad2deg,
    rad2deg_list,
    parse_safety_status_bits,
    parse_robot_mode,
    rtde_c_batch_monitor,
    rtde_d_batch_monitor,
    rtde_r_batch_monitor,
    set_real_time_priority,
)

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../ur_control')))
from vacuumScripts import Vacuum_grip, Vacuum_release


def print_parse_safety_status_bits(safety_status):
    ret = parse_safety_status_bits(safety_status)
    print(f"IS_NORMAL_MODE={ret['IS_NORMAL_MODE']}")
    print(f"IS_REDUCED_MODE={ret['IS_REDUCED_MODE']}")
    print(f"IS_PROTECTIVE_STOPPED={ret['IS_PROTECTIVE_STOPPED']}")
    print(f"IS_RECOVERY_MODE={ret['IS_RECOVERY_MODE']}")
    print(f"IS_SAFEGUARD_STOPPED={ret['IS_SAFEGUARD_STOPPED']}")
    print(f"IS_SYSTEM_EMERGENCY_STOPPED={ret['IS_SYSTEM_EMERGENCY_STOPPED']}")
    print(f"IS_ROBOT_EMERGENCY_STOPPED={ret['IS_ROBOT_EMERGENCY_STOPPED']}")
    print(f"IS_EMERGENCY_STOPPED={ret['IS_EMERGENCY_STOPPED']}")
    print(f"IS_VIOLATION={ret['IS_VIOLATION']}")
    print(f"IS_FAULT={ret['IS_FAULT']}")
    print(f"IS_STOPPED_DUE_TO_SAFETY={ret['IS_STOPPED_DUE_TO_SAFETY']}")


def print_parse_robot_mode(robot_mode):
    print(f"parsed_robot_mode={parse_robot_mode(robot_mode)}")

## Parameters

# Simulated robot
# robot_ip = "127.0.0.1"
robot_ip = "10.5.5.102"
# Real robot

velocity = 0.5
acceleration = 0.5
# -1.0 means use the robot’s default frequency, 500Hz for e-Series and UR-Series, while its 125Hz for the CB-series
rtde_frequency = 500.0
dt = 1.0 / rtde_frequency  # 2ms
use_custom_script = False
if use_custom_script:
    flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_CUSTOM_SCRIPT | RTDEControl.FLAG_NO_WAIT
else:
    flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
# The port used for the External URCap interface (default: 50002) 
# RTDEは30004
ur_cap_port = 50002  # default
lookahead_time = 0.1
gain = 300
# ur_rtde realtime priorities
# プログラムのrt_app_priorityとないはず
# controlが相対的にreceiveより優位なのには意味がある
rt_receive_priority = 90
rt_control_priority = 85
verbose = True
use_upper_range_registers = False
dashboard_port = 29999

# hostname, frequency, flags, ur_cap_port, rt_priority
# 結構頻繁に偶発的に
# RTDEControlInterface Exception: Traceback (most recent call last):
# std::bad_alloc
#   File "/home/uclab/UR_remote_codes/UR_MQTT_Control/src/tests/my_example.py", line 51, in <module>
#     rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
#              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# RuntimeError: ur_rtde: Failed to start control script, before timeout of 5 seconds
# となる
# その他エラー例
# RTDEControlInterface: realtime priority set successfully!
# Traceback (most recent call last):
#   File "/home/uclab/UR_remote_codes/UR_MQTT_Control/src/tests/my_example.py", line 101, in <module>
#     rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
#              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# RuntimeError: ur_rtde: Please enable remote control on the robot!
# その他エラー例
# RTDEControlInterface: realtime priority set successfully!
# Connected successfully to: 10.5.5.102 at 30004
# Waiting for RTDE data synchronization to start...
# RTDE synchronization started
# Segmentation fault
max_trials = 3
trial = 0
while True:
    try:
        rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
        break
    except Exception as e:
        print(f"Exception during RTDEControl initialization: {e}")
        print("Retrying in 2 seconds...")
        time.sleep(2)
        trial += 1
        if trial >= max_trials:
            print("Max trials reached. Exiting.")
            sys.exit(1)

if use_custom_script:
    # エラーが出る
    # script_path = os.path.join(os.path.dirname(__file__), "../ur_control/scripts/generated/ePick_control.script")
    # エラーは出ないが動きはしない
    script_path = os.path.join(os.path.dirname(__file__), "../ur_control/scripts/generated/rtde_control_original.script")
    if not os.path.exists(script_path):
        raise FileNotFoundError(f"Custom script file not found: {script_path}")
    # from script_client import ScriptClient
    # rtde_sc = ScriptClient(robot_ip, 5, 17, 30003, True)
    # # NOTE: ソースコードのpybind部分が引数をうけつけないバグになっている
    # rtde_sc.setScriptFile(script_path)
    # corrected_script = rtde_sc.getScript()
    # script_path = os.path.join(os.path.dirname(__file__), "../ur_control/scripts/generated/rtde_control_original_corrected.script")
    # with open(script_path, "w") as f:
    #     f.write(corrected_script)
    rtde_c.setCustomScriptFile(script_path)
# hostname, verbose, use_upper_range_registers
rtde_io = RTDEIO(robot_ip, verbose, use_upper_range_registers)
# hostname, frequency, variables, verbose, use_upper_range_registers, rt_priority
# variables: A vector of variable names to be monitored (empty vector means use all default variables)
# verbose: Enable verbose output for debugging purposes.
# use_upper_range_registers: ? realtime_control_example.pyではFalse
rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], verbose, use_upper_range_registers, rt_receive_priority)

# hostname, port, verbose
rtde_d = DashboardClient(robot_ip, dashboard_port, verbose)
# 初期化時点では接続されない。connectではじめて接続される
# timeout_ms
rtde_d.connect(2000)
# 意図的にポートを変えるとエラーを送出する
# rtde_d = DashboardClient(robot_ip, 50000, verbose)
# RuntimeError: Timeout connecting to UR dashboard server.
# rtde_d.connect(2000)
# rtde_d.isConnected()=True
print(f"{rtde_d.isConnected()=}")
# rtde_d.isConnected()=False
rtde_d.disconnect()
print(f"{rtde_d.isConnected()=}")
rtde_d.connect(2000)
# rtde_d.isConnected()=True
print(f"{rtde_d.isConnected()=}")
# Powers on the robot arm.
# rtde_d.powerOn()
# # Powers off the robot arm.
# rtde_d.powerOff()
# # Powers off the robot arm. ドキュメントにはこうあるが間違い。正しくはEnable the robot arm.に相当
# rtde_d.brakeRelease()
# Closes the current popup and unlocks protective stop.
# The unlock protective stop command fails with an exception if less than 5 seconds has passed since the protective stop occurred.
# rtde_d.unlockProtectiveStop()
# Use this when robot gets a safety fault or violation to restart the safety.
# After safety has been rebooted the robot will be in Power Off.
# You should always ensure it is okay to restart the system. 
# It is highly recommended to check the error log before using this command 
# (either via PolyScope or e.g. ssh connection).
# このコメントによればエラーログをRTDEで確認する方法がない?
# rtde_d.restartSafety()
# rtde_d.isInRemoteControl()=True
print(f"{rtde_d.isInRemoteControl()=}")
# Returns the remote control status of the robot.
# If the robot is in remote control it returns false and if remote control is disabled or robot is in local control it returns false.
# Closes connection.
# rtde_d.disconnect()
# rtde_d.quit()
# Shuts down and turns off robot and controller.
# rtde_d.shutdown()

# この有無でgetActualTCPForce()の値が変わるか確認する
# This function is used for enabling and disabling the use of external F/T measurements in the controller.
# これをservoStop -> MoveJの間に入れるとProtective Stop (C207A0: Fieldbus input disconnected)となる
# servoJの前でも駄目 (同じエラー)
# enable = True
# is_success = rtde_c.ftRtdeInputEnable(enable)
# print(f"rtde_c.ftRtdeInputEnable={is_success}")

# # Zeroes the TCP force/torque measurement from the builtin force/torque sensor by subtracting the current measurement from the subsequent.
# is_success = rtde_c.zeroFtSensor()
# print(f"rtde_c.zeroFtSensor={is_success}")

# # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# # Generalized forces in the TCP
# print(f"{rtde_r.getActualTCPForce()=}")
# # [-14476.894071003719, 12678.536316641186, 28045.929934306776, 263.8146426475125, 280.7545354497713, -343.4272562644521]
# # Get the raw force and torque measurement, not compensated for forces and torques caused by the payload.
# print(f"{rtde_r.getFtRawWrench()=}")

# この順番で指定する
# Set application real-time priority
set_real_time_priority()

# 現在の作業台でのデフォルト姿勢
default_joints = [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
# J6だけ回転
new_joints = [0.0, -90.0, -90.0, -90.0, 90.0, 90.0]

# 関節移動
# デフォルトではSync（Asyncではない）
# Move to joint position (linear in joint-space)
# Parameters:
#         q – joint positions
#         speed – joint speed of leading axis [rad/s]. defaults to 1.05
#         acceleration – joint acceleration of leading axis [rad/s^2]. defaults to 1.4
#         asynchronous – a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function. Default is false, this means the function will block until the movement has completed.
print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")
print(f"{rad2deg_list(rtde_r.getActualQ())=}")
print(f"{rtde_c.moveJ(deg2rad_list(new_joints))=}")
print(f"{rad2deg_list(rtde_r.getActualQ())=}")
print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")
print(f"{rad2deg_list(rtde_r.getActualQ())=}")

if False:
    def send_grip():
        t_start = time.time()
        print(f"{rtde_c.sendCustomScript(Vacuum_grip)=}")
        print(f"Time taken for Vacuum_grip: {time.time() - t_start} seconds")

    def send_release():
        t_start = time.time()
        print(f"{rtde_c.sendCustomScript(Vacuum_release)=}")
        print(f"Time taken for Vacuum_release: {time.time() - t_start} seconds")

    thread = threading.Thread(target=send_grip)
    thread.start()
    thread.join()
    thread = threading.Thread(target=send_release)
    thread.start()
    thread.join()

    print(f"{rad2deg_list(rtde_r.getActualQ())=}")

    rtde_c.stopScript()
    sys.exit(0)

# TCP移動
pose = rtde_r.getActualTCPPose()
# X, Y, Z, Rx, Ry, Rz
# 単位はmとrad
# ツールのTCP位置をTPのInstallationに登録した場合
# [0.492758731909551, -0.1328801058413286, 0.19206447671314636, -2.2212006216673545, 2.2207765897624494, -0.00123055083415886]
# 登録しない場合
# [0.492452086206032, -0.13287208068971268, 0.4880439642897617, -2.2212206768130365, 2.220815840844321, -0.001218890328266665]
print(f"{rtde_r.getActualTCPPose()=}")
# Zを10cm上げる
pose[2] += 0.1
# Move to position (linear in tool-space)
# Parameters:
#         pose – target pose
#         speed – tool speed [m/s]. defaults to 0.25
#         acceleration – tool acceleration [m/s^2]. defaults to 1.2
#         asynchronous – a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function. Default is false, this means the function will block until the movement has completed.
# rtde_c.moveL(pose)=True
print(f"{rtde_c.moveL(pose)=}")
# rtde_r.getActualTCPPose()=[0.49273930108863656, -0.13288742454175062, 0.2920007388438206, -2.2212821775287495, 2.2207783698084587, -0.0011734765814475432]
print(f"{rtde_r.getActualTCPPose()=}")
print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")

if False:
    # ひじ特異姿勢になり、途中で止まってしまい以下の出力が得られる。処理は以降も継続するが
    # ROBOT_MODE_POWER_OFF、IS_FAULT、IS_STOPPED_DUE_TO_SAFETYとなる
    # IS_PROTECTIVE_STOPPEDではないことに注意
    # 以下のようにもなる
    # rtde_r.isProtectiveStopped()=False
    # rtde_c.isProgramRunning()=False
    # rtde_c.getRobotStatus()=0
    # rtde_r.getRobotStatus()=0
    # rtde_r.isEmergencyStopped()=False
    # 再接続などしても復帰しない
    # TPはずっと以下のようになる
    # Safety Message
    # Fault
    # ShoulderA: C306A3: joint: Acceleration failed to pass sanity check
    # Explanation
    # The received joint acceleration target is invalid.
    # Suggestion
    # Try the following actions to see which resolves the issue:
    # (A) Adjusts the robot program to reduce peak acceleration and torques,
    # (B) Conduct a complete rebooting sequence,
    # (C) Update software,
    # (D) Contact your local Universal Robots service provider for assistance
    # -> Go to initialization screen ボタン
    # -> Restart ボタン
    # -> Power OFF > Booting > Robot Idle > Brake Release > Robot Operational の順にTPで操作
    # 必要に応じて特異姿勢から関節制御などで抜けておく
    # 重要なことはこの加速度エラーになるとPower OFFまで状態が戻るということ。自動復帰する場合はここからの操作が必要
    pose[2] += 1
    # rtde_c.moveL(pose)=False
    print(f"{rtde_c.moveL(pose)=}")
    # rtde_r.getActualTCPPose()=[0.4924559063506938, -0.13288871284280165, 0.4838238083626931, 2.221603472340064, -2.2209961733083947, -5.609133893831176e-05]
    print(f"{rtde_r.getActualTCPPose()=}")
    time.sleep(1)
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))
    # # rtde_d.safetystatus()='Safetystatus: FAULT'
    # print(f"{rtde_d.safetystatus()=}")
    # # rtde_d.programState()='STOPPED <unnamed>'
    # print(f"{rtde_d.programState()=}")
    # # rtde_d.robotmode()='Robotmode: POWER_OFF'
    # print(f"{rtde_d.robotmode()=}")
    # # IS_NORMAL_MODE=False
    # # IS_REDUCED_MODE=False
    # # IS_PROTECTIVE_STOPPED=False
    # # IS_RECOVERY_MODE=False
    # # IS_SAFEGUARD_STOPPED=False
    # # IS_SYSTEM_EMERGENCY_STOPPED=False
    # # IS_ROBOT_EMERGENCY_STOPPED=False
    # # IS_EMERGENCY_STOPPED=False
    # # IS_VIOLATION=False
    # # IS_FAULT=True
    # # IS_STOPPED_DUE_TO_SAFETY=True
    # print_parse_safety_status_bits(rtde_r.getSafetyStatusBits())
    # # parsed_robot_mode=ROBOT_MODE_POWER_OFF
    # print_parse_robot_mode(rtde_r.getRobotMode())
    # TODO: Popupを閉じるのはいいがちゃんとPopupの情報を保存したい
    # TODO: どれを選ぶ?上ほど対象範囲が広い? -> そうではない
    # Closes the popup.
    # C306A3: joint: Acceleration failed to pass sanity checkでは検出できない
    # print(f"{rtde_d.closePopup()=}")
    # Closes a safety popup.
    # C306A3: joint: Acceleration failed to pass sanity checkは検出可能
    print(f"{rtde_d.closeSafetyPopup()=}")
    # Closes the current popup and unlocks protective stop.
    # The unlock protective stop command fails with an exception 
    # if less than 5 seconds has passed since the protective stop occurred.
    # C306A3: joint: Acceleration failed to pass sanity checkでは検出できない
    # print(f"{rtde_d.unlockProtectiveStop()=}")
    # Use this when robot gets a safety fault or violation to restart the safety.
    # After safety has been rebooted the robot will be in Power Off.
    # Attention
    #     You should always ensure it is okay to restart the system. 
    #     It is highly recommended to check the error log before using 
    #     this command (either via PolyScope or e.g. ssh connection).
    # かなり時間を空ければ自動復帰できる
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))
    time.sleep(10)
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))
    print(f"{rtde_d.restartSafety()=}")
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))
    # RTDEReceiveInterface Exception: RTDEControlInterface: Could not receive data from robot...
    # RTDEControlInterface Exception: Operation canceledOperation canceled

    # Reconnecting...
    # RTDEControlInterface: Robot is disconnected, reconnecting...
    # Connected successfully to: 10.5.5.102 at 30004
    # Waiting for RTDE data synchronization to start...
    # RTDE synchronization started
    # RTDEControlInterface Exception: ur_rtde: Failed to start control script, before timeout of 5 seconds
    # Connected successfully to: 10.5.5.102 at 30004
    # RTDE synchronization started
    time.sleep(10)
    while not rtde_r.isConnected():
        time.sleep(1)
        try:
            print(f"{rtde_r.reconnect()=}")
            print("Reconnect in loop")
        except Exception as e:
            print(f"Exception during rtde_r.reconnect(): {e}")
    while not rtde_c.isConnected():
        time.sleep(1)
        try:
            print(f"{rtde_c.reconnect()=}")
            print("Reconnect in loop")
        except Exception as e:
            print(f"Exception during rtde_c.reconnect(): {e}")
    print(f"{rtde_d.powerOn()=}")
    # time.sleep(10)
    # RTDEControlInterface Exception: ur_rtde: Failed to start control script, before timeout of 5 seconds
    print(f"{rtde_d.brakeRelease()=}")
    time.sleep(15)
    # IS_NORMAL_MODE=True
    # IS_REDUCED_MODE=False
    # IS_PROTECTIVE_STOPPED=False
    # IS_RECOVERY_MODE=False
    # IS_SAFEGUARD_STOPPED=False
    # IS_SYSTEM_EMERGENCY_STOPPED=False
    # IS_ROBOT_EMERGENCY_STOPPED=False
    # IS_EMERGENCY_STOPPED=False
    # IS_VIOLATION=False
    # IS_FAULT=False
    # IS_STOPPED_DUE_TO_SAFETY=False
    print_parse_safety_status_bits(rtde_r.getSafetyStatusBits())
    # ここで既にRUNNINGになっているが、control scriptがnot running
    # parsed_robot_mode=ROBOT_MODE_RUNNING
    print_parse_robot_mode(rtde_r.getRobotMode())
    # ここでは既にconnectされている
    while not rtde_c.isConnected():
        time.sleep(1)
        try:
            print(f"{rtde_c.reconnect()=}")
        except Exception as e:
            print(f"Exception during rtde_c.reconnect(): {e}")
    print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")
    rtde_c.stopScript()
    # rtde_c.disconnect()
    del rtde_c
    import gc
    gc.collect()
    time.sleep(10)
    # The RTDE Control script has been re-uploaded.
    # rtde_c.reuploadScript()=True
    # print(f"{rtde_c.reuploadScript()=}")
    rtde_c = None
    while True:
        if rtde_c is None or not rtde_c.isConnected():
            try:
                rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
            except Exception as e:
                print(f"Exception during RTDEControl re-initialization: {e}")
                time.sleep(1)
        else:
            break
    time.sleep(10)
    # NOTE: この方法で自動復帰できたが、もう少しスマートな方法があるかもしれない
    # →もう少し試行錯誤したがなさそう
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))
    print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")
    # rtde_c.stopScript()
    # sys.exit(0)

## Protective Stopの場合
if False:
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))

    # Triggers a protective stop on the robot. Can be used for testing and debugging.
    print(f"{rtde_c.triggerProtectiveStop()=}")
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))

    time.sleep(10)
    # print(f"{rtde_d.closePopup()=}")
    # print(f"{rtde_d.closeSafetyPopup()=}")
    # ｌ接続できない
    print(f"{rtde_d.unlockProtectiveStop()=}")
    # print(f"{rtde_d.restartSafety()=}")

    time.sleep(5)

    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))

    print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")

    rtde_c.stopScript()
    del rtde_c
    import gc
    gc.collect()
    # time.sleep(10)
    # こうすれば接続できる
    rtde_c = None
    while True:
        if rtde_c is None or not rtde_c.isConnected():
            try:
                rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
            except Exception as e:
                print(f"Exception during RTDEControl re-initialization: {e}")
                time.sleep(1)
        else:
            break
    print(rtde_c_batch_monitor(rtde_c))
    print(rtde_d_batch_monitor(rtde_d))
    print(rtde_r_batch_monitor(rtde_r))

    print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")


    rtde_c.stopScript()
    sys.exit(0)

## サーボモード

# Move to initial joint position with a regular moveJ
joint_q = default_joints.copy()
print(f"{rtde_c.moveJ(deg2rad_list(joint_q))=}")

rtde_io.setInputIntRegister(18, 1)
# 0.01では駄目だった
time.sleep(0.1)
rtde_io.setInputIntRegister(18, 0)
time.sleep(5)
rtde_io.setInputIntRegister(18, 2)
time.sleep(0.1)
rtde_io.setInputIntRegister(18, 0)
time.sleep(5)

if False:
    # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
    # 合計45度回転させる
    for i in range(1000):
        # This function is used in combination with waitPeriod() and is used to get the start of a control period / cycle. 
        t_start = rtde_c.initPeriod()
        # Servo to position (linear in joint-space)
        # Parameters:
        #         q – joint positions [rad]
        #         speed – NOT used in current version
        #         acceleration – NOT used in current version
        #         time – time where the command is controlling the robot. The function is blocking for time t [S]
        #         lookahead_time – time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
        #         gain – proportional gain for following target position, range [100,2000]
        # あまりlookahead_time, gainを変えてもわからなそう
        rtde_c.servoJ(deg2rad_list(joint_q), velocity, acceleration, dt, lookahead_time, gain)
        joint_q[0] += 0.045
        joint_q[5] += 0.045
        # Used for waiting the rest of the control period, set implicitly as dt = 1 / frequency. 
        # the function is to be used in combination with the initPeriod().
        # dtとrtde_frequencyは厳密には一緒じゃなくても可能
        rtde_c.waitPeriod(t_start)
        if i % 200 == 0:
            print(f"{rtde_io.setInputIntRegister(18, 1)=}")
        elif (i + 100) % 200 == 0:
            print(f"{rtde_io.setInputIntRegister(18, 2)=}")
        else:
            print(f"{rtde_io.setInputIntRegister(18, 0)=}")

    # 無いと、
    # TPで、Another thread is already controlling the robot
    # 標準出力で、RTDEControlInterface: RTDE control script is not running!
    # となる（プログラム側はしたがって例外は発生しない）
    # Stop servo mode and decelerate the robot.
    # a – rate of deceleration of the tool [m/s^2]
    rtde_c.servoStop()

# Receive側のループ例
for i in range(5):
    t_start = rtde_r.initPeriod()
    print(f"{t_start=}")
    rtde_r.waitPeriod(t_start)

# BEGIN

# Checks if the given joint position is reachable and within the current safety 
# limits of the robot.
# This check considers joint limits (if the target pose is specified as joint positions), 
# safety planes limits, TCP orientation deviation limits and range of the robot. 
# If a solution is found when applying the inverse kinematics to the given target 
# TCP pose, this pose is considered reachable
# 使えるが制御ループで毎回呼ぶのは重いかもしれない
print(f"{rtde_c.isJointsWithinSafetyLimits(deg2rad_list(default_joints))=}")

robot_mode = rtde_r.getRobotMode()
# parsed_robot_mode=ROBOT_MODE_RUNNING
print_parse_robot_mode(robot_mode)

safety_status = rtde_r.getSafetyStatusBits()
# IS_NORMAL_MODE=True
# IS_REDUCED_MODE=False
# IS_PROTECTIVE_STOPPED=False
# IS_RECOVERY_MODE=False
# IS_SAFEGUARD_STOPPED=False
# IS_SYSTEM_EMERGENCY_STOPPED=False
# IS_ROBOT_EMERGENCY_STOPPED=False
# IS_EMERGENCY_STOPPED=False
# IS_VIOLATION=False
# IS_FAULT=False
# IS_STOPPED_DUE_TO_SAFETY=False
print_parse_safety_status_bits(safety_status)

# rtde_r.isProtectiveStopped()=False
print(f"{rtde_r.isProtectiveStopped()=}")
# どう復帰する?
# Triggers a protective stop on the robot. Can be used for testing and debugging.
# print(rtde_c.triggerProtectiveStop())
# Returns true if a program is running on the controller, 
# otherwise it returns false This is just a shortcut for getRobotStatus() & RobotStatus::ROBOT_STATUS_PROGRAM_RUNNING.
# rtde_c.isProgramRunning()=True
print(f"{rtde_c.isProgramRunning()=}")
# Robot status Bits 0-3:
# Is power on | Is program running | Is teach button pressed | Is power button pressed
# There is a synchronization gap between the three interfaces RTDE Control RTDE Receive and Dashboard Client.
# RTDE Control and RTDE Receive open its own RTDE connection and so the internal 
# state is not in sync. That means, if RTDE Control reports, that program is running, 
# RTDE Receive may still return that program is not running. 
# The update of the Dashboard Client even needs more time. 
# That means, the dashboard client still returns program not running after some 
# milliseconds have passed after RTDE Control already reports program running.
# rtde_c.getRobotStatus()=3
# 011 -> Power on + Program running
print(f"{rtde_c.getRobotStatus()=}")
# rtde_r.getRobotStatus()=3
# 011 -> Power on + Program running
print(f"{rtde_r.getRobotStatus()=}")
# rtde_r.isEmergencyStopped()=False
print(f"{rtde_r.isEmergencyStopped()=}")

# 以下の出力を確認
# RTDE - Socket disconnected
rtde_r.disconnect()
# Noneなどが入る可能性がある?
# rad2deg_list(rtde_r.getActualQ())=[42.971001351733, -89.99351376829648, -89.99537846641007, -89.99753296109706, 89.98502389962833, 42.98963752288338]
print(f"{rad2deg_list(rtde_r.getActualQ())=}")
# rtde_r.isConnected()=False
print(f"{rtde_r.isConnected()=}")
# rtde_c.isConnected()=True
print(f"{rtde_c.isConnected()=}")
# 以下の出力を確認
# Connected successfully to: 10.5.5.102 at 30004
# RTDE synchronization started
rtde_r.reconnect()
# rad2deg_list(rtde_r.getActualQ())=[43.00390237296495, -89.99949496081511, -89.99858865531998, -89.99986340796816, 90.00009129693738, 43.00297005214325]
print(f"{rad2deg_list(rtde_r.getActualQ())=}")
# rtde_r.isConnected()=True
print(f"{rtde_r.isConnected()=}")
# rtde_c.isConnected()=True
print(f"{rtde_c.isConnected()=}")
# END

# time.sleep(1)
# TODO: なぜか
# rtde_rを切ることでrtde_cも切れるのか
# rtde_rは再接続されたはずだが
# RTDEControlInterface: RTDE control script is not running!
# rtde_c.moveJ(deg2rad_list(default_joints))=False
print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")

# BEGIN
# RTDE - Socket disconnected
# rtde_c.disconnect()=None
print(f"{rtde_c.disconnect()=}")
# RTDEControlInterface: RTDE control script is not running!
# # rtde_c.moveJ(deg2rad_list(default_joints))=False
print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")
# rtde_c.isConnected()=False
print(f"{rtde_c.isConnected()=}")
# Connected successfully to: 10.5.5.102 at 30004
# RTDE synchronization started
# Waiting for RTDE data synchronization to start...
# Traceback (most recent call last):
#   File "/home/uclab/UR_remote_codes/UR_MQTT_Control/src/tests/my_example.py", line 294, in <module>
#     rtde_c.reconnect()
# RuntimeError: ur_rtde: Failed to start control script, before timeout of 5 seconds
# エラーが出ることがある。control scriptをスタートしていない状態が全体的に問題
# 偶発的に以下のエラーが出ることもある
# Dashboard client deadline expired
# Traceback (most recent call last):
#   File "/home/uclab/UR_remote_codes/UR_MQTT_Control/src/tests/my_example.py", line 363, in <module>
#     print(f"{rtde_c.reconnect()=}")
#              ^^^^^^^^^^^^^^^^^^
# RuntimeError: Timeout connecting to UR dashboard server.
print(f"{rtde_c.reconnect()=}")
# RTDE - Socket disconnected
print(f"{rtde_c.moveJ(deg2rad_list(default_joints))=}")
# RTDE - Socket disconnected
print(f"{rtde_c.isConnected()=}")
# ここまででプログラムがハングすることはない
# END

print(f"{rtde_c.isConnected()=}")
# This function will terminate the script on controller.
print(f"{rtde_c.stopScript()=}")

rtde_d.disconnect()
