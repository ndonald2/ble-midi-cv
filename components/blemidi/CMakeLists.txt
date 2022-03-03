idf_component_register(SRCS "blemidi.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash console)

set(BLEMIDI_DEVICE_NAME "BLE-MIDI-CV Proto")
target_compile_definitions(${COMPONENT_LIB} PUBLIC BLEMIDI_DEVICE_NAME="${BLEMIDI_DEVICE_NAME}")