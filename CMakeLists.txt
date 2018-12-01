project(learn_vo_1)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(
"/usr/include/eigen3"
${OpenCV_INCLUDE_DIRS}
)
add_executable(learn_vo_1 main.cpp)
target_link_libraries(learn_vo_1 ${OpenCV_LIBS})


