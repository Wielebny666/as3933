set(SOURCES as3933_hal.c
			as3933.c)

idf_component_register(SRCS "${SOURCES}"
					INCLUDE_DIRS .
					REQUIRES 	foreach 
								spi_master_ext)
