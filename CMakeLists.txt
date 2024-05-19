cmake_minimum_required(VERSION 3.5)
project(robot1)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/cmd_pid.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  ${PROJECT_NAME}/imagem42.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ${PROJECT_NAME}/simple_pid DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mapeamento.py
  ${PROJECT_NAME}/db.py
  ${PROJECT_NAME}/robot_navigator.py
  ${PROJECT_NAME}/machines.py
  ${PROJECT_NAME}/insert_data.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
