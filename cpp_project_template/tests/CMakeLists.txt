cmake_minimum_required(VERSION 3.20)

include_directories(../include)

find_package(GTest REQUIRED)
enable_testing()

macro(make_units_test name)
    add_executable(${name} t_${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main prp_lib)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

make_units_test(example)
