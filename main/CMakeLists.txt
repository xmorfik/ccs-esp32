idf_component_register(SRCS "main.c"
                            "rest_server.c"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
