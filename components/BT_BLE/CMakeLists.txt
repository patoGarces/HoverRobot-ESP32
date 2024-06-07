idf_component_register(SRCS "BT_BLE.c"
                    REQUIRES bt driver nvs_flash
                    INCLUDE_DIRS "include")
