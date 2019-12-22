set(COMPONENT_SRCS entry_point.c)
register_component()
target_link_libraries(${COMPONENT_LIB} ${FIRMWARE_LIB})



add_library(idf_shim_lib shim_freertos.c)
target_link_libraries(idf_shim_lib ${COMPONENT_LIB})
target_link_libraries(${COMPONENT_LIB} idf_shim_lib)
