cmake_minimum_required(VERSION 3.15)

set(DRFL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/vendor/API-DRFL/include)
set(DRFL_LIB_DIR ${CMAKE_SOURCE_DIR}/src/vendor/API-DRFL/lib)

# test.cpp用の実行ファイルターゲット
add_executable(test.app test.cpp)
target_include_directories(test.app PRIVATE ${DRFL_INCLUDE_DIR})
target_link_libraries(test.app PRIVATE
    ${DRFL_LIB_DIR}/libDRFL.a
    ${DRFL_LIB_DIR}/libPocoFoundation.so.80
    ${DRFL_LIB_DIR}/libPocoNet.so.80
)
set_target_properties(test.app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    BUILD_RPATH "${DRFL_LIB_DIR}"
    INSTALL_RPATH "${DRFL_LIB_DIR}"
)
install(TARGETS test.app RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
