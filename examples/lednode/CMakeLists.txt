nuttx_add_application(
  NAME lednode
  SRCS lednode_main.c
  STACKSIZE ${CONFIG_DEFAULT_TASK_STACKSIZE}
)
