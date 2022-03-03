#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "blemidi.h"

#define TAG "ble-midi-cv"

void callback_midi_message_received(uint8_t blemidi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
    ESP_LOGI(TAG, "CALLBACK blemidi_port=%d, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", blemidi_port, timestamp, midi_status, len, continued_sysex_pos);
}

void app_main(void)
{
    int status = blemidi_init(callback_midi_message_received);
    if (status < 0)
    {
        ESP_LOGE(TAG, "BLE MIDI Driver returned status=%d", status);
    }
    else
    {
        ESP_LOGI(TAG, "BLE MIDI Driver initialized successfully");
    }

    esp_log_level_set(TAG, ESP_LOG_INFO);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}