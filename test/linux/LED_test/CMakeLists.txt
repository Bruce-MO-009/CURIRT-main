set(SOURCES rt_led_test.c)

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


add_executable(rt_led_test ${SOURCES})
target_link_libraries(rt_led_test soem /usr/xenomai/lib/libalchemy.so /usr/xenomai/lib/libcopperplate.so)
install(TARGETS rt_led_test DESTINATION bin)

set(SOURCES LED_test.c)
add_executable(LED_test ${SOURCES})
target_link_libraries(LED_test soem)
install(TARGETS LED_test DESTINATION bin)
