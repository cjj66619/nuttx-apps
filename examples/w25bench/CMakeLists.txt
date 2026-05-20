# ##############################################################################
# apps/examples/w25bench/CMakeLists.txt
# ##############################################################################

if(CONFIG_EXAMPLES_W25BENCH)
  nuttx_add_application(
    MODULE
    ${CONFIG_EXAMPLES_W25BENCH}
    NAME
    ${CONFIG_EXAMPLES_W25BENCH_PROGNAME}
    SRCS
    w25bench_main.c
    STACKSIZE
    ${CONFIG_EXAMPLES_W25BENCH_STACKSIZE}
    PRIORITY
    ${CONFIG_EXAMPLES_W25BENCH_PRIORITY})
endif()
