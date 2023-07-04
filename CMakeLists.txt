cmake_minimum_required(VERSION 3.12)
project(WebcamStreaming)

set(CMAKE_CXX_STANDARD 11)

# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

add_executable(WebcamStreaming v4l2cam.cpp)

# OpenCV 라이브러리 링크
target_link_libraries(WebcamStreaming ${OpenCV_LIBS})
