idf_component_register(SRCS "OTA_SERVER.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_server app_update esp_timer json
)

target_add_binary_data(
    ${COMPONENT_LIB}
    "${CMAKE_CURRENT_LIST_DIR}/web/index.html"
    BINARY
)

target_add_binary_data(
    ${COMPONENT_LIB}
    "${CMAKE_CURRENT_LIST_DIR}/web/style.css"
    BINARY
)
