#include <iostream>
#include "../vendor/API-DRFL/include/DRFLEx.h"

using namespace DRAFramework;

int main() {
    // 動作確認用
    CDRFLEx drfl;
    std::cout << "API-DRFL test: CDRFLEx instance created." << std::endl;
    // ここでAPI-DRFLの関数を呼び出して動作確認ができます。
    // 例: バージョン取得
    std::cout << "Version: " << drfl.get_library_version() << std::endl;
    return 0;
}
