cmake_minimum_required(VERSION 3.5)
project(tracer_description)

# C++14 이상 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 패키지 의존성
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Include directories
include_directories(
  include
)

# 예시: 라이브러리 생성
# add_library(${PROJECT_NAME} src/tracer_description.cpp)

# 예시: 실행 파일 생성
# add_executable(${PROJECT_NAME}_node src/tracer_description_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   urdf
#   xacro
# )

# 설치
 install(DIRECTORY rviz
   DESTINATION share/${PROJECT_NAME}
 )
install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
 )
 install(DIRECTORY urdf
   DESTINATION share/${PROJECT_NAME}
 )
 install(DIRECTORY meshes
   DESTINATION share/${PROJECT_NAME}
 )

# ament 패키지 등록
ament_package()

