cmake_minimum_required(VERSION 3.15)

find_package(pybind11 REQUIRED)

set(DRFL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/vendor/API-DRFL/include)
set(DRFL_LIB_DIR ${CMAKE_SOURCE_DIR}/src/vendor/API-DRFL/lib)

# pybind11モジュールターゲット
pybind11_add_module(doosan_robot doosan_robot.cpp)
target_include_directories(doosan_robot PRIVATE ${DRFL_INCLUDE_DIR})

# 静的ライブラリと共有ライブラリのフルパスを指定
target_link_libraries(doosan_robot PRIVATE 
    ${DRFL_LIB_DIR}/libDRFL.a
    ${DRFL_LIB_DIR}/libPocoFoundation.so.80
    ${DRFL_LIB_DIR}/libPocoNet.so.80
)

# 実行時に共有ライブラリを見つけられるようにRPATHを設定
set_target_properties(doosan_robot PROPERTIES
    BUILD_RPATH "${DRFL_LIB_DIR}"
    INSTALL_RPATH "${DRFL_LIB_DIR}"
)

install(TARGETS doosan_robot LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
