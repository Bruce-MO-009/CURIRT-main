set(SOURCES motor_control.c)
add_executable(motor_control ${SOURCES})
target_link_libraries(motor_control soem)
install(TARGETS motor_control DESTINATION bin)
