cmake_minimum_required(VERSION 4.2)
project(estica_solutions)

set(CMAKE_CXX_STANDARD 14)

add_executable(estica_solutions
        src/cpp_pubsub/src/publisher_member_function.cpp
        src/cpp_pubsub/src/subscriber_member_function.cpp
        src/hello_cpp/src/publisher.cpp
        src/hello_cpp/src/subscriber.cpp)
