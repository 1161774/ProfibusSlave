set(SRCS
    "ProfibusSlaveEmulator.c"
    "Application/VSD.c"
    "Application/ET200S.c"
    "Profibus/Protocol.c"
    "Profibus/Slave.c"
    "Profibus/Controller.c"
    "Serial/Serial.c"
    "Logging/pb_log.c"
    "Utility/list.c"
)

if(CONFIG_WIFI_ENABLED)
    list(APPEND SRCS
        "Network/WiFi.c"
        "Network/SNTP.c"
    )
endif()

idf_component_register(SRCS ${SRCS} INCLUDE_DIRS ".")
