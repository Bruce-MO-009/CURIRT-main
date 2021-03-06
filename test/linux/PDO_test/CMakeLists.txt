set(SOURCES position_control.c)


include_directories(/usr/xenomai/include/alchemy/)
include_directories(/usr/xenomai/include/)
#include_directories(/usr/xenomai/include/rtdm)

include_directories(/usr/xenomai/cobalt/)
include_directories(/usr/xenomai/posix/)
include_directories(/usr/xenomai/boilerplate/)

link_directories(/usr/xenomai/lib)

set(xeno_cflags_params "--skin=posix" "--cflags")
execute_process(
    COMMAND xeno-config ${xeno_cflags_params}
    OUTPUT_VARIABLE xeno_cflags
    OUTPUT_STRIP_TRAILING_WHITESPACE)

set(xeno_ldflags_params "--skin=posix" "--ldflags")
execute_process(
    COMMAND xeno-config ${xeno_ldflags_params}
    OUTPUT_VARIABLE xeno_ldflags
    OUTPUT_STRIP_TRAILING_WHITESPACE)

# Compiler and linker options
set(CMAKE_C_FLAGS          "${CMAKE_CXX_FLAGS} ${xeno_cflags}")
set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} ${xeno_cflags}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${xeno_ldflags}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${xeno_ldflags}")


add_executable(position_control ${SOURCES})
target_link_libraries(position_control soem)
install(TARGETS position_control DESTINATION bin)

set(SOURCES velocity_control.c)
add_executable(velocity_control ${SOURCES})
target_link_libraries(velocity_control soem /usr/xenomai/lib/libalchemy.so /usr/xenomai/lib/libcopperplate.so)
install(TARGETS velocity_control DESTINATION bin)

set(SOURCES torque_control.c)
add_executable(torque_control ${SOURCES})
target_link_libraries(torque_control soem /usr/xenomai/lib/libalchemy.so /usr/xenomai/lib/libcopperplate.so)
install(TARGETS torque_control DESTINATION bin)

set(SOURCES torque_control2.c)
add_executable(torque_control2 ${SOURCES})
target_link_libraries(torque_control2 soem /usr/xenomai/lib/libalchemy.so /usr/xenomai/lib/libcopperplate.so)
install(TARGETS torque_control2 DESTINATION bin)

set(SOURCES position_show.c)
add_executable(position_show ${SOURCES})
target_link_libraries(position_show soem)
install(TARGETS position_show DESTINATION bin)
