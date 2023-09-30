idf_component_register(
    SRCS 
        "rs485_example.c" 
        "Application/VSD.c"
        "Profibus/Protocol.c"
        "Profibus/Slave.c"
        "Profibus/Controller.c"
        "Utility/list.c"
        "UART/uart.c"
    INCLUDE_DIRS 
        "."
)

