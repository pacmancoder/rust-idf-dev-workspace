set(COMPONENT_SRCS entry_point.c)
register_component()
target_link_libraries(${COMPONENT_NAME} ${FIRMWARE_LIB})
