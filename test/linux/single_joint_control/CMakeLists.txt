set(SOURCES single_joint_control.c)
add_executable(single_joint_control ${SOURCES})
target_link_libraries(single_joint_control soem)
install(TARGETS single_joint_control DESTINATION bin)
