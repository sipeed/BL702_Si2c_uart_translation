set(TARGET_REQUIRED_SRCS ${CMAKE_CURRENT_LIST_DIR}/i2c_slave.c
${CMAKE_CURRENT_LIST_DIR}/uart_interface.c)
set(mains main.c)
generate_bin()



