#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <functional>

#include "../vendor/API-DRFL/include/DRFLEx.h"
using namespace DRAFramework;

#include <assert.h>
#include "doosan_robot.hpp"
#include <sstream>
#include <vector>
#include <queue>
#include <mutex>
#include <tuple>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace {
  CDRFLEx Drfl;
  bool g_bHasControlAuthority = FALSE;
  bool g_TpInitailizingComplted = FALSE;

  enum class LogMode { None, Cpp, Queue };
  LogMode g_log = LogMode::None;
  // タプル: (timestamp, level, message)
  std::queue<std::tuple<double, std::string, std::string>> g_log_queue;
  std::mutex g_log_queue_mutex;
  constexpr size_t g_log_queue_max_size = 500;

  void push_log(const std::string& msg, const std::string& level = "INFO") {
    if (g_log == LogMode::None) return;
    if (g_log == LogMode::Cpp) {
      std::cout << msg << std::endl;
      return;
    }
    if (g_log == LogMode::Queue) {
      // Unix timestampを取得
      auto now = std::chrono::system_clock::now();
      auto duration = now.time_since_epoch();
      double timestamp = std::chrono::duration<double>(duration).count();
      
      std::lock_guard<std::mutex> lock(g_log_queue_mutex);
      if (g_log_queue.size() >= g_log_queue_max_size) g_log_queue.pop();
      g_log_queue.push(std::make_tuple(timestamp, level, msg));
    }
  }
}


// TP初期化の際に一度だけ呼ばれる
void OnTpInitializingCompleted() {
  g_TpInitailizingComplted = TRUE;
  // TPが初期化されたら制御権を要求する
  Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
  push_log("[OnTpInitializingCompleted] MANAGE_ACCESS_CONTROL_FORCE_REQUEST");
}

// エラーなどでプログラムが終了したときに完全に終了したかをチェックする
void OnProgramStopped(const PROGRAM_STOP_CAUSE eStopCause) {
  switch (eStopCause) {
    case PROGRAM_STOP_CAUSE_NORMAL:
      push_log("[OnProgramStopped] PROGRAM_STOP_CAUSE_NORMAL");
      break;
    case PROGRAM_STOP_CAUSE_FORCE:
      push_log("[OnProgramStopped] PROGRAM_STOP_CAUSE_FORCE");
      break;
    case PROGRAM_STOP_CAUSE_ERROR:
      push_log("[OnProgramStopped] PROGRAM_STOP_CAUSE_ERROR");
      break;
    default:
      break;
  }
  // プログラムをゆっくりストップする
  assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
  push_log("[OnProgramStopped] PlayDrlStop");
}

// 操作状態は15種類あり、11種類はこのコールバックかget_robot_stateで確認できる (残り4種はreservedでユーザーには重要でない)
void OnMonitoringStateCB(const ROBOT_STATE eState) {
  switch ((unsigned char)eState) {
    // プログラムを先に起動してTPを後で起動する場合にTPの起動前の状態
    case STATE_NOT_READY:
      push_log("[OnMonitoringStateCB] STATE_NOT_READY");
      break;
    // プログラムを先に起動してTPを後で起動する場合にTPの起動中の状態
    case STATE_INITIALIZING:
      push_log("[OnMonitoringStateCB] STATE_INITIALIZING");
      break;
    case STATE_EMERGENCY_STOP:
      push_log("[OnMonitoringStateCB] STATE_EMERGENCY_STOP");
      break;
    // 正常に起動した場合、充分待つとこの状態になる
    case STATE_STANDBY:
      push_log("[OnMonitoringStateCB] STATE_STANDBY");
      break;
    case STATE_MOVING:
      push_log("[OnMonitoringStateCB] STATE_MOVING");
      break;
    case STATE_TEACHING:
      push_log("[OnMonitoringStateCB] STATE_TEACHING");
      break;
    // モーター電源は入ったままでのセーフティストップ状態
    case STATE_SAFE_STOP:
      push_log("[OnMonitoringStateCB] STATE_SAFE_STOP");
      break;
    // モーター電源から入れ直すに必要のあるセーフティストップ状態
    case STATE_SAFE_OFF:
      push_log("[OnMonitoringStateCB] STATE_SAFE_OFF");
      break;
    // リカバリモードに入ってからの復帰が必要なセーフティストップ状態
    // 遠隔操作する場合はリカバリモードで手動でロボットを動かす作業は難しいので
    // セーフティストップを解除することはできないのでプログラム上は何もしない
    case STATE_SAFE_STOP2:
      push_log("[OnMonitoringStateCB] STATE_SAFE_STOP2");
      break;
    // リカバリモードに入ってからの復帰が必要な
    // モーター電源から入れ直すに必要のあるセセーフティストップ状態
    // 遠隔操作する場合はリカバリモードで手動でロボットを動かす作業は難しいので
    // セーフティストップを解除することはできないのでプログラム上は何もしない
    case STATE_SAFE_OFF2:
      push_log("[OnMonitoringStateCB] STATE_SAFE_OFF2");
      break;
    // リカバリ状態。リカバリ完了せずに脱出しても意味がないのでプログラム上は何もしない
    case STATE_RECOVERY:
      push_log("[OnMonitoringStateCB] STATE_RECOVERY");
      break;
    default:
      break;
  }
  return;
}

void RecoverFromRecoverableRobotState(const ROBOT_STATE eState) {
  switch ((unsigned char)eState) {
    // モーター電源は入ったままでのセーフティストップ状態
    case STATE_SAFE_STOP:
      push_log("[RecoverFromRecoverableRobotState] STATE_SAFE_STOP");
      if (g_bHasControlAuthority) {
        // 状態をセーフティストップを元に戻した状態にする
        // STATE_SAFE_STOP -> STATE_STANDBY
        Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
        push_log("[RecoverFromRecoverableRobotState] CONTROL_RESET_SAFET_STOP");
      }
      break;
    // モーター電源から入れ直すに必要のあるセーフティストップ状態
    case STATE_SAFE_OFF:
      push_log("[RecoverFromRecoverableRobotState] STATE_SAFE_OFF");
      if (g_bHasControlAuthority) {
        // モーター電源を入れ直す
        Drfl.SetRobotControl(CONTROL_SERVO_ON);
        push_log("[RecoverFromRecoverableRobotState] CONTROL_SERVO_ON");
      }
      break;
    default:
      break;
  }
  return;
}

// 取得済みの状態Aと取得していない状態Bに分類できる
void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
  switch (eTrasnsitControl) {
    // 制御権の移行をリクエストされたら (おそらくTPなどから)
    case MONITORING_ACCESS_CONTROL_REQUEST:
      push_log("[OnMonitroingAccessControlCB] MONITORING_ACCESS_CONTROL_REQUEST");
      assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
      break;
    // 制御権を失ったら (おそらくTPなどが制御権を失うケースを含む)
    case MONITORING_ACCESS_CONTROL_LOSS:
      push_log("[OnMonitroingAccessControlCB] MONITORING_ACCESS_CONTROL_LOSS");
      g_bHasControlAuthority = FALSE;
      if (g_TpInitailizingComplted) {
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
      }
      break;
    // 制御権を取得したことを確認したら
    case MONITORING_ACCESS_CONTROL_GRANT:
      push_log("[OnMonitroingAccessControlCB] MONITORING_ACCESS_CONTROL_GRANT");
      g_bHasControlAuthority = TRUE;
      break;
    // 制御権の移行を拒否されたら
    case MONITORING_ACCESS_CONTROL_DENY:
      push_log("[OnMonitroingAccessControlCB] MONITORING_ACCESS_CONTROL_DENY");
      break;
    default:
      break;
  }
}

// ログレベル
// - 1: info
// - 2: warning (operational error)
// - 3: error (safety or device error)
// ログクラス番号
// - 1: framework
// - 2: algorithm
// - 3: gui
// - 4: inverter board
// - 5: safety board
// メッセージ番号
// - DRSC.hを参照
void OnLogAlarm(LPLOG_ALARM tLog) {
  std::stringstream ss;
  ss << "[OnLogAlarm] {";
  ss << "\"level\":" << (unsigned int)tLog->_iLevel << ",";
  ss << "\"group\":" << (unsigned int)tLog->_iGroup << ",";
  ss << "\"index\":" << tLog->_iIndex << ",";
  ss << "\"param_0\":\"" << std::string(tLog->_szParam[0]) << "\",";
  ss << "\"param_1\":\"" << std::string(tLog->_szParam[1]) << "\",";
  ss << "\"param_2\":\"" << std::string(tLog->_szParam[2]) << "\"}";
  push_log(ss.str());
}

// TPのポップアップメッセージを表示する
void OnTpPopup(LPMESSAGE_POPUP tPopup) {
  std::stringstream ss;
  ss << "[OnTpPopup] {";
  ss << "\"popup_message\":\"" << tPopup->_szText << "\",";
  ss << "\"message_level\":" << tPopup->_iLevel << ",";
  ss << "\"button_type\":" << tPopup->_iBtnType << "}";
  push_log(ss.str());
}

// TPのログを表示する
void OnTpLog(const char* strLog) {
  std::stringstream ss;
  ss << "[OnTpLog] {";
  ss << "\"log_message\":\"" << strLog << "\"}";
  push_log(ss.str());
}

void OnDisConnected(const std::string & ip) {
  while (!Drfl.open_connection(ip)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

Robot::Robot(const std::string & ip, const std::string & log, float fPeriod): ip_(ip), log_(log), fPeriod_(fPeriod) {
  push_log("[Robot] start");
  if (log == "none") g_log = LogMode::None;
  else if (log == "cpp") g_log = LogMode::Cpp;
  else if (log == "queue") g_log = LogMode::Queue;
  else g_log = LogMode::None;
  // 起動のためのコールバック
  // 現在の状態の出力と異常な状態からの復帰
  // HACK: 無名関数を関数ポインタにするにはキャプチャ式を使えないのでtrue/falseをベタ書きしている
  // https://rinatz.github.io/cpp-book/ch02-09-lambda-expressions/
  Drfl.set_on_monitoring_state([](ROBOT_STATE e){OnMonitoringStateCB(e);});
  // 制御権を取得するまでの一連の処理を背後で実行
  Drfl.set_on_monitoring_access_control([](MONITORING_ACCESS_CONTROL e){OnMonitroingAccessControlCB(e);});
  // TP初期化の際に一度だけ呼ばれる
  Drfl.set_on_tp_initializing_completed([](){OnTpInitializingCompleted();});
  // エラー時のためのコールバック
  Drfl.set_on_log_alarm([](LPLOG_ALARM e){OnLogAlarm(e);});
  Drfl.set_on_tp_popup([](LPMESSAGE_POPUP e){OnTpPopup(e);});
  Drfl.set_on_tp_log([](const char *strLog){OnTpLog(strLog);});
  Drfl.set_on_program_stopped([](PROGRAM_STOP_CAUSE e){OnProgramStopped(e);});
  // TODO: OnDisConnectedにipを渡す方法
  // OnDisConnectedに重要な処理は書いていないので現状何もしない
  // Drfl.set_on_disconnected([ip](){OnDisConnected(true, ip);});
}

Robot::~Robot() {
  push_log("[~Robot] start");
  leave_servo_mode();
  disable();
  stop();
}

bool Robot::start() {
  push_log("[start] start");
  if (!Drfl.open_connection(ip_)) {
    push_log("[start] open_connection failed");
    return false;
  }
  // バージョンを指定する必要あり
  if (!Drfl.setup_monitoring_version(1)) {
    push_log("[start] setup_monitoring_version failed");
    return false;
  }
  // 自動サーボOFFを解除
  if (!Drfl.set_auto_servo_off(0, 0)) {
    push_log("[start] set_auto_servo_off failed");
    return false;
  }
  // リアルタイム制御ポートに接続
  // 特に受信のためにstart内で接続する必要がある
  // enableしなくても有効に受信できることを確認済み
  // 1対1接続しかできない
  push_log("[start] connect_rt_control");
  if (!Drfl.connect_rt_control(ip_)) return false;
  // PCからロボットコントローラへの送信設定
  // 送信データはトルクやDigital/Analog Input/Outputであり
  // servo制御ではないので基本使わない
  // fPeriod周期の入力がnLossCnt個こなければリアルタイム制御はdisconnectされる
  // nLossCnt = -1はdisconnect判定しない
  // push_log("[start] set_rt_control_input");
  // if (!Drfl.set_rt_control_input("v1.0", fPeriod_, -1)) return;
  // ロボットコントローラーからPCへの送信設定
  // set_rt_control_outputは現在はnLossCntによるdisconnect判定はないとのこと
  push_log("[start] set_rt_control_output");
  // TODO: disconnect判定後の対応
  if (!Drfl.set_rt_control_output("v1.0", fPeriod_, 4)) return false;
  // データ送受信開始
  push_log("[start] start_rt_control");
  if (!Drfl.start_rt_control()) return false;
  // 直後はデータ受信できないので少し待つ
  std::this_thread::sleep_for(std::chrono::seconds(1));
  is_started_ = true;
  return true;
}
bool Robot::enable() {
  push_log("[enable] start");
  if (!Drfl.set_robot_control(CONTROL_SERVO_ON)) {
    push_log("[enable] set_robot_control failed");
    return false;
  }
  // 初期化完了して制御を受け付ける状態になっていないか、
  // 制御権がない場合はOnMonitroingAccessControlCBが
  // 背後で実行されるのを待機する
  int retry = 0;
  int max_retry = 30;
  while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority) {
    this_thread::sleep_for(std::chrono::milliseconds(1000));
    push_log("[enable] Taking control...");
    retry += 1;
    if (retry == max_retry) {
      push_log("[enable] Taking control retry failed");
      return false;
    }
  }
  // 自動モードと手動モードを切り替える
  if (!Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
    push_log("[enable] set_robot_mode failed");
    return false;
  }
  // シミュレーションでなく実機
  // ROBOT_SYSTEM_REAL or ROBOT_SYSTEM_VIRTUAL
  if (!Drfl.set_robot_system(ROBOT_SYSTEM_REAL)) {
    push_log("[enable] set_robot_system failed");
    return false;
  }
  is_enabled_ = true;
  return true;
}
void Robot::move_pose(float x, float y, float z, float rx, float ry, float rz) {
  push_log("[move_pose] start");
  float x1[6] = {x, y, z, rx, ry, rz};
  // xyz、rxryrzの速度、加速度を指定
  float tvel[2] = { 50, 50 };
  float tacc[2] = { 100, 100 };
  // 到達時間は0で自動計算
  float ttime = 0;
  while (true) {
    if(Drfl.check_motion() == 0)
    {
      Drfl.movel(x1, tvel, tacc, ttime);
      break;
    }
  }
  push_log("[move_pose] end");
}
void Robot::move_joint(float x, float y, float z, float rx, float ry, float rz) {
  push_log("[move_joint] start");
  float x1[6] = {x, y, z, rx, ry, rz};
  // 速度、加速度
  float tvel = 10;
  float tacc = 20;
  // 到達時間は0で自動計算
  float ttime = 0;
  while (true) {
    if(Drfl.check_motion() == 0)
    {
      Drfl.movej(x1, tvel, tacc, ttime);
      break;
    }
  }
  push_log("[move_joint] end");
}
void Robot::move_default_pose_until_completion() {
  push_log("[move_default_pose_until_completion] start");
  float* x1 = default_pose_.data();
  // xyz、rxryrzの速度、加速度を指定
  float tvel[2] = { 50, 50 };
  float tacc[2] = { 100, 100 };
  // 到達時間は0で自動計算
  float ttime = 0;
  while (true) {
    if(Drfl.check_motion() == 0)
    {
      Drfl.movel(x1, tvel, tacc, ttime);
      break;
    }
  }
  push_log("[move_default_pose_until_completion] end");
}
std::vector<float> Robot::get_default_pose() {
  return default_pose_;
}
std::vector<double> Robot::get_current_joint() {
  LPROBOT_POSE lpPose = Drfl.get_current_posj();
  std::vector<double> ret;
  for (int k = 0; k < NUM_JOINT; k++) {
    ret.emplace_back(lpPose->_fPosition[k]);
  }
  return ret;
}
std::vector<double> Robot::get_current_pose() {
  ROBOT_TASK_POSE* posx = Drfl.get_current_posx();
  std::vector<double> ret;
  for (int k = 0; k < NUM_TASK; k++) {
    ret.emplace_back(posx->_fTargetPos[k]);
  }
  return ret;
}
std::vector<double> Robot::get_current_pose_rt() {
  LPRT_OUTPUT_DATA_LIST tData = Drfl.read_data_rt();
  std::vector<double> ret;
  double time_stamp = tData->time_stamp;
  ret.emplace_back(time_stamp);
  float *pose =  tData->actual_tcp_position;
  for (int i = 0; i <= 5; i++) 
  {
    ret.emplace_back(pose[i]);
  }
  return ret;
}
std::vector<double> Robot::get_current_joint_rt() {
  LPRT_OUTPUT_DATA_LIST tData = Drfl.read_data_rt();
  std::vector<double> ret;
  double time_stamp = tData->time_stamp;
  ret.emplace_back(time_stamp);
  float *pose =  tData->actual_joint_position;
  for (int i = 0; i <= 5; i++) 
  {
    ret.emplace_back(pose[i]);
  }
  return ret;
}
std::vector<double> Robot::get_current_pose_vel_rt() {
  LPRT_OUTPUT_DATA_LIST tData = Drfl.read_data_rt();
  std::vector<double> ret;
  double time_stamp = tData->time_stamp;
  ret.emplace_back(time_stamp);
  float *pose =  tData->actual_tcp_velocity;
  for (int i = 0; i <= 5; i++) 
  {
    ret.emplace_back(pose[i]);
  }
  return ret;
}
std::vector<double> Robot::get_current_external_tcp_force_rt() {
  LPRT_OUTPUT_DATA_LIST tData = Drfl.read_data_rt();
  std::vector<double> ret;
  double time_stamp = tData->time_stamp;
  ret.emplace_back(time_stamp);
  // estimated tcp force w.r.t. base coordinates [N, Nm]
  float *pose =  tData->external_tcp_force;
  for (int i = 0; i <= 5; i++) 
  {
    ret.emplace_back(pose[i]);
  }
  return ret;
}
bool Robot::enter_servo_mode() {
  is_in_servo_mode_ = true;
  return true;
}
void Robot::move_pose_servo_by_pos(float x, float y, float z, float rx, float ry, float rz)
{
  // ドキュメントによれば、位置制御は未完成
  // 20 [ms]以上の間隔であれば問題ないとのことだが、
  // 1 [ms]で制御したい場合は、位置制御より速度制御のほうが推奨される
  float fTargetPos[6] = {x, y, z, rx, ry, rz};
  // -10000で自動設定になる
  float fTargetVel[6] = {-10000, -10000, -10000, -10000, -10000, -10000};
  float fTargetAcc[6] = {-10000, -10000, -10000, -10000, -10000, -10000};
  float fTargetTime = fPeriod_;
  Drfl.servol_rt(fTargetPos, fTargetVel, fTargetAcc, fTargetTime);
}
void Robot::move_pose_servo_by_vel(float x, float y, float z, float rx, float ry, float rz) {
  // 速度制御。単位: [mm/s, deg/s]
  float fTargetVel[6] = {x, y, z, rx, ry, rz};
  // -10000で自動設定になる
  float fTargetAcc[6] = {-10000, -10000, -10000, -10000, -10000, -10000};
  Drfl.speedl_rt(fTargetVel, fTargetAcc, fPeriod_);
}
bool Robot::move_joint_servo_by_vel(float x, float y, float z, float rx, float ry, float rz) {
  // 非同期
  // 速度制御。単位: [deg/s]
  float fTargetVel[6] = {x, y, z, rx, ry, rz};
  // -10000で自動設定になる
  float fTargetAcc[6] = {-10000, -10000, -10000, -10000, -10000, -10000};
  return Drfl.speedj_rt(fTargetVel, fTargetAcc, fPeriod_);
}
bool Robot::leave_servo_mode() {
  if (is_in_servo_mode_) {
    is_in_servo_mode_ = false;
  }
  return true;
}
bool Robot::disable() {
  // Doosanの場合はディスエーブル処理は不要
  if (is_enabled_) {
    if (!Drfl.servo_off(STOP_TYPE_SLOW)) {
      push_log("[disable] set_robot_control failed");
      return false;
    }
    is_enabled_ = false;
  }
  return true;
}
bool Robot::stop() {
  if (is_started_) {
    // データ送受信終了
    push_log("[leave_servo_mode] stop_rt_control");
    if (!Drfl.stop_rt_control()) return false;
    // リアルタイム制御解除
    push_log("[leave_servo_mode] disconnect_rt_control");
    if (!Drfl.disconnect_rt_control()) return false;
    Drfl.CloseConnection();
    is_started_ = false;
  }
  return true;
}
ROBOT_STATE Robot::get_robot_state() {
  return Drfl.get_robot_state();
}

void Robot::recover_from_recoverable_robot_state() {
  ROBOT_STATE current_state = Drfl.get_robot_state();
  RecoverFromRecoverableRobotState(current_state);
}

std::vector<std::tuple<double, std::string, std::string>> Robot::pop_log_queue() {
  std::vector<std::tuple<double, std::string, std::string>> logs;
  std::lock_guard<std::mutex> lock(g_log_queue_mutex);
  while (!g_log_queue.empty()) {
    logs.push_back(g_log_queue.front());
    g_log_queue.pop();
  }
  return logs;
}

namespace py = pybind11;

PYBIND11_MODULE(doosan_robot, m)
{
    m.doc() = "pybind11 example plugin";

    py::enum_<ROBOT_STATE>(m, "ROBOT_STATE")
        .value("STATE_INITIALIZING", STATE_INITIALIZING)
        .value("STATE_STANDBY", STATE_STANDBY)
        .value("STATE_MOVING", STATE_MOVING)
        .value("STATE_SAFE_OFF", STATE_SAFE_OFF)
        .value("STATE_TEACHING", STATE_TEACHING)
        .value("STATE_SAFE_STOP", STATE_SAFE_STOP)
        .value("STATE_EMERGENCY_STOP", STATE_EMERGENCY_STOP)
        .value("STATE_HOMMING", STATE_HOMMING)
        .value("STATE_RECOVERY", STATE_RECOVERY)
        .value("STATE_SAFE_STOP2", STATE_SAFE_STOP2)
        .value("STATE_SAFE_OFF2", STATE_SAFE_OFF2)
        .value("STATE_RESERVED1", STATE_RESERVED1)
        .value("STATE_RESERVED2", STATE_RESERVED2)
        .value("STATE_RESERVED3", STATE_RESERVED3)
        .value("STATE_RESERVED4", STATE_RESERVED4)
        .value("STATE_NOT_READY", STATE_NOT_READY)
        .value("STATE_LAST", STATE_LAST)
        .export_values();
    
    py::class_<Robot>(m, "DoosanRobot")
        .def(py::init<const std::string &, const std::string &, float>(),
             py::arg("ip") = "192.168.5.43", py::arg("log") = "none", py::arg("fPeriod") = 0.001)
        .def("start", &Robot::start)
        .def("enable", &Robot::enable)
        .def("move_pose", &Robot::move_pose)
        .def("move_joint", &Robot::move_joint)
        .def("move_default_pose_until_completion", &Robot::move_default_pose_until_completion)
        .def("get_default_pose", &Robot::get_default_pose)
        .def("get_current_pose", &Robot::get_current_pose)
        .def("get_current_joint", &Robot::get_current_joint)
        .def("get_current_pose_rt", &Robot::get_current_pose_rt)
        .def("get_current_pose_vel_rt", &Robot::get_current_pose_vel_rt)
        .def("get_current_joint_rt", &Robot::get_current_joint_rt)
        .def("get_current_external_tcp_force_rt", &Robot::get_current_external_tcp_force_rt)
        .def("enter_servo_mode", &Robot::enter_servo_mode)
        .def("move_pose_servo_by_pos", &Robot::move_pose_servo_by_pos)
        .def("move_pose_servo_by_vel", &Robot::move_pose_servo_by_vel)
        .def("move_joint_servo_by_vel", &Robot::move_joint_servo_by_vel)
        .def("leave_servo_mode", &Robot::leave_servo_mode)
        .def("disable", &Robot::disable)
        .def("stop", &Robot::stop)
        .def("get_robot_state", &Robot::get_robot_state)
        .def("recover_from_recoverable_robot_state", &Robot::recover_from_recoverable_robot_state)
        .def("pop_log_queue", &Robot::pop_log_queue);
}
