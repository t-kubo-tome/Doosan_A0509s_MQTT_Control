#pragma once
#include <string>
#include <vector>
#include <tuple>
#include "../vendor/API-DRFL/include/DRFC.h"



class Robot
{
public:
  Robot(
    const std::string & ip = "192.168.5.43",
    const std::string & log = "none",
    float fPeriod = 0.001
  );
  ~Robot();
  bool start();
  bool enable();
  bool move_pose(float x, float y, float z, float rx, float ry, float rz);
  bool move_joint(float x, float y, float z, float rx, float ry, float rz);
  bool move_default_pose_until_completion();
  std::vector<double> get_current_pose();
  std::vector<double> get_current_joint();
  std::vector<float> get_default_pose();
  std::vector<double> get_current_pose_rt();
  std::vector<double> get_current_joint_rt();
  std::vector<double> get_current_pose_vel_rt();
  std::vector<double> get_current_external_tcp_force_rt();
  bool move_pose_servo_by_pos(float x, float y, float z, float rx, float ry, float rz);
  bool move_pose_servo_by_vel(float x, float y, float z, float rx, float ry, float rz);
  bool move_joint_servo_by_pos(float j1, float j2, float j3, float j4, float j5, float j6);
  bool move_joint_servo_by_vel(float j1, float j2, float j3, float j4, float j5, float j6);
  bool disable();
  bool stop();
  ROBOT_STATE get_robot_state();
  void recover_from_recoverable_robot_state();

  // ログキュー操作
  std::vector<std::tuple<double, std::string, std::string>> pop_log_queue();

private:
  float fPeriod_ {-1};
  std::vector<float> default_pose_ { -550, -50, 400, 5, -135, -5 };
  std::string log_;
  const std::string ip_;
  bool is_in_servo_mode_ {false};
  bool is_enabled_ {false};
  bool is_started_ {false};
};
