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
#include <sstream>
#include <vector>
#include <queue>
#include <mutex>

namespace {
  CDRFLEx Drfl;
  bool g_bHasControlAuthority = FALSE;
  bool g_TpInitailizingComplted = FALSE;
  float fPeriod = 0.001;  // s

  enum class LogMode { None, Cpp, Queue };
  LogMode g_log = LogMode::None;
  std::queue<std::string> g_log_queue;
  std::mutex g_log_queue_mutex;
  constexpr size_t g_log_queue_max_size = 500;

  void push_log(const std::string& msg) {
    if (g_log == LogMode::None) return;
    if (g_log == LogMode::Cpp) {
      std::cout << msg << std::endl;
      return;
    }
    if (g_log == LogMode::Queue) {
      std::lock_guard<std::mutex> lock(g_log_queue_mutex);
      if (g_log_queue.size() >= g_log_queue_max_size) g_log_queue.pop();
      g_log_queue.push(msg);
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

bool _main() {
  std::cout << "API-DRFL test: CDRFLEx instance created." << std::endl;
  // ここでAPI-DRFLの関数を呼び出して動作確認ができます。
  // 例: バージョン取得
  std::cout << "Version: " << Drfl.get_library_version() << std::endl;
  std::string ip_ = "192.168.5.43";
  Drfl.set_on_tp_initializing_completed([](){OnTpInitializingCompleted();});
  if (!Drfl.open_connection(ip_)) {
    push_log("[start] open_connection failed");
    return false;
  }
  // バージョンを指定する必要あり
  if (!Drfl.setup_monitoring_version(1)) {
    push_log("[start] setup_monitoring_version failed");
    return false;
  }
  if (!Drfl.open_connection(ip_)) {
    push_log("[start] open_connection failed 2");
    return false;
  }
  // バージョンを指定する必要あり
  if (!Drfl.setup_monitoring_version(1)) {
  push_log("[start] setup_monitoring_version failed 2");
    return false;
  }
  return true;
}

int main() {
  g_log = LogMode::Cpp;
  bool ret = _main();
  // ret to string
  std::string ret_str = ret ? "true" : "false";
  push_log(ret_str);
  return  ret ? 0 : 1;
}
