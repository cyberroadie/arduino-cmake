set(ARDUINO_DEFAULT_BOARD atmega328) # Default Board ID, when not specified
set(ARDUINO_DEFAULT_PORT /dev/tty.usbserial-A900acNo) # Default Port, when not specified

generate_arduino_library(adafruit_neopixel_lib
    SRCS adafruit_neopixel/Adafruit_NeoPixel.cpp
    HDRS adafruit_neopixel/Adafruit_NeoPixel.h)

generate_arduino_library(adafruit_neomatrix_lib
    SRCS adafruit_neomatrix/Adafruit_NeoMatrix.cpp adafruit_neomatrix/extras/gamma.c
    HDRS adafruit_neomatrix/Adafruit_NeoMatrix.h adafruit_neomatrix/gamma.h adafruit_gfx/Adafruit_GFX.h adafruit_neopixel/Adafruit_NeoPixel.h)

generate_arduino_library(adafruit_gfx_lib
    SRCS adafruit_gfx/Adafruit_GFX.cpp adafruit_gfx/glcdfont.c
    HDRS adafruit_gfx/Adafruit_GFX.h)

set(BLE_UART_SRCS adafruit_ble_uart/utility/aci_setup.cpp
                  adafruit_ble_uart/utility/acilib.cpp
                  adafruit_ble_uart/utility/hal_aci_tl.cpp
                  adafruit_ble_uart/utility/lib_aci.cpp)

set(BLE_UART_HDRS adafruit_ble_uart/utility/aci.h
                  adafruit_ble_uart/utility/aci_cmds.h
                  adafruit_ble_uart/utility/aci_evts.h
                  adafruit_ble_uart/utility/aci_protocol_defines.h
                  adafruit_ble_uart/utility/aci_setup.h
                  adafruit_ble_uart/utility/acilib.h
                  adafruit_ble_uart/utility/acilib_defs.h
                  adafruit_ble_uart/utility/acilib_if.h
                  adafruit_ble_uart/utility/acilib_types.h
                  adafruit_ble_uart/utility/ble_system.h
                  adafruit_ble_uart/utility/dtm.h
                  adafruit_ble_uart/utility/hal_platform.h
                  adafruit_ble_uart/utility/lib_aci.h
                  adafruit_ble_uart/utility/hal/hal_io.h
                  adafruit_ble_uart/utility/hal/hal_aci_tl.h
                  adafruit_ble_uart/utility/uart/services.h)

generate_arduino_library(adafruit_ble_uart_lib
    SRCS adafruit_ble_uart/Adafruit_BLE_UART.cpp ${BLE_UART_SRCS}
    HDRS adafruit_ble_uart/Adafruit_BLE_UART.h ${BLE_UART_HDRS})

generate_arduino_library(guggenhat_lib
    SRCS guggenhat_lib.cpp
    HDRS guggenhat_lib.h)

generate_arduino_firmware(guggenhat
    SRCS guggenhat.cpp
    LIBS guggenhat_lib adafruit_neopixel_lib adafruit_neomatrix_lib adafruit_gfx_lib adafruit_ble_uart_lib)