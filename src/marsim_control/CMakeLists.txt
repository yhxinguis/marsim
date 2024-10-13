cmake_minimum_required(VERSION 3.5)
project(marsim_control)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python programs
install(PROGRAMS scripts/bluerov2_pid_pose_z_control.py DESTINATION lib/${PROJECT_NAME} RENAME bluerov2_pid_pose_z_control)

# Create symlinks for Python scripts
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/bluerov2_pid_pose_z_control
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/bluerov2_pid_pose_z_control.py)")

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()