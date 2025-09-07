set(srcs
    main.c
    speech_commands_action.c
    )

set(requires
    hardware_driver
    bt
    )

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS include
                       REQUIRES ${requires}
                       PRIV_REQUIRES ws2812_control nvs_flash)
