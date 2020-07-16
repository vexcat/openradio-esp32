idf_component_register(SRCS "main.cpp" "openradio.cpp" "crc.cpp" "rtos_common.cpp" "rtos_common.hpp" "openradio.hpp"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)