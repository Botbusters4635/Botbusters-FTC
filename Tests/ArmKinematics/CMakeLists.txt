cmake_minimum_required(VERSION 3.15)
project(Test)

set(TARGET_LIBS sfml-graphics sfml-window sfml-system pthread GL)
link_libraries(${TARGET_LIBS})

include_directories(Test src)

file(GLOB_RECURSE Test_SRC
        "src/*.cpp"
        "src/**/*.cpp"
)

set(CMAKE_CXX_STANDARD 14)
add_executable(Test ${Test_SRC})

