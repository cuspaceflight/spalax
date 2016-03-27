set (SOURCES ${SOURCES}
    PARENT_SCOPE)

set (HEADERS ${HEADERS}
	${CMAKE_CURRENT_SOURCE_DIR}/avionics_config.h
	${CMAKE_CURRENT_SOURCE_DIR}/component_state_config.h
	${CMAKE_CURRENT_SOURCE_DIR}/messaging_config.h
	${CMAKE_CURRENT_SOURCE_DIR}/telemetry_config.h
    PARENT_SCOPE)
