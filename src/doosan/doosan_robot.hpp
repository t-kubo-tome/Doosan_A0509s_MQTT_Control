#pragma once
#include <string>
#include <vector>



class Robot
{
public:
  Robot(
    const std::string & ip = "192.168.5.43",
    const std::string & log = "none"
  );
  ~Robot();
  bool start();
  bool enable();
  void move_pose(float x, float y, float z, float rx, float ry, float rz);
  void move_joint(float x, float y, float z, float rx, float ry, float rz);
  void move_default_pose_until_completion();
  std::vector<double> get_current_pose();
  std::vector<double> get_current_joint();
  std::vector<float> get_default_pose();
  std::vector<double> get_current_pose_rt();
  std::vector<double> get_current_joint_rt();
  std::vector<double> get_current_pose_vel_rt();
  bool enter_servo_mode(float fPeriod = 0.001);
  void move_pose_servo_by_pos(float x, float y, float z, float rx, float ry, float rz);
  void move_pose_servo_by_vel(float x, float y, float z, float rx, float ry, float rz);
  void move_joint_servo_by_vel(float x, float y, float z, float rx, float ry, float rz);
  bool leave_servo_mode();
  bool disable();
  bool stop();

  // ログキュー操作
  std::vector<std::string> pop_log_queue();

private:
  float fPeriod_ {-1};
  std::vector<float> default_pose_ { -550, -50, 400, 5, -135, -5 };
  std::string log_;
  const std::string ip_;
  bool is_in_servo_mode_ {false};
  bool is_enabled_ {false};
  bool is_started_ {false};
};
