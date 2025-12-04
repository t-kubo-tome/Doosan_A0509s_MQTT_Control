cmake_minimum_required(VERSION 3.15)
project(Doosan_A0509s_Control)

add_subdirectory(src/doosan)

# testsディレクトリが存在する場合のみビルド
if(EXISTS ${CMAKE_SOURCE_DIR}/src/tests/CMakeLists.txt)
    add_subdirectory(src/tests)
endif()
