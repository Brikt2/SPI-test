cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(dmx_led_driver C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


# RGB er standard.
# Alternativ: RGBW.

set(LED_FORMAT "RGB" CACHE STRING "LED pixel format")

set_property(
    CACHE LED_FORMAT
    PROPERTY STRINGS RGB RGBW
)


if(NOT LED_FORMAT STREQUAL "RGB"
   AND NOT LED_FORMAT STREQUAL "RGBW")

    message(FATAL_ERROR "LED_FORMAT must be RGB or RGBW")

endif()


pico_sdk_init()


add_executable(dmx_led_driver

    main.c

)


# Generer C-header fra PIO-programmet.

pico_generate_pio_header(

    dmx_led_driver

    ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio

)


# Velg LED-format.

if(LED_FORMAT STREQUAL "RGB")

    target_compile_definitions(

        dmx_led_driver PRIVATE LED_FORMAT_RGB=1

    )

else()

    target_compile_definitions(

        dmx_led_driver PRIVATE LED_FORMAT_RGB=0

    )

endif()


# Biblioteker.

target_link_libraries(

    dmx_led_driver

    pico_stdlib

    hardware_clocks

    hardware_dma

    hardware_gpio

    hardware_irq

    hardware_pio

    hardware_uart

)


# USB serial for feilsøking.

pico_enable_stdio_usb(dmx_led_driver 1)

pico_enable_stdio_uart(dmx_led_driver 0)


# Generer UF2-fil.

pico_add_extra_outputs(dmx_led_driver)
