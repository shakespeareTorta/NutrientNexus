cmake_minimum_required(VERSION 3.8)
project(my_tb3_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
