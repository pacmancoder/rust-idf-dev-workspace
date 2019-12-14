set(COMPONENT_SRCS entry_point.c)
register_component()
target_link_libraries(${COMPONENT_LIB} ${FIRMWARE_LIB})
target_link_options(${COMPONENT_LIB} PUBLIC "-mlongcalls" "-mtext-section-literals" "-Wa,--auto-litpools")
