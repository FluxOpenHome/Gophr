set(SRC_DIRS ".")
set(PRIV_INCLUDE_DIRS ".")

# Add esp-matter common utils if available
if(DEFINED ENV{ESP_MATTER_PATH})
    set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
    list(APPEND PRIV_INCLUDE_DIRS "${ESP_MATTER_PATH}/examples/common/utils")
endif()

idf_component_register(
    SRC_DIRS ${SRC_DIRS}
    PRIV_INCLUDE_DIRS ${PRIV_INCLUDE_DIRS}
)

# Matter requires C++17
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
