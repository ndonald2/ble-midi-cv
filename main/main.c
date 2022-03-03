#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_log.h"
#include "linenoise/linenoise.h"
#include "blemidi.h"

#define TAG "ble-midi-cv"
#define PROMPT_STR CONFIG_IDF_TARGET

void callback_midi_message_received(uint8_t blemidi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
    ESP_LOGI(TAG, "CALLBACK blemidi_port=%d, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", blemidi_port, timestamp, midi_status, len, continued_sysex_pos);
}

void start_console(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT_STR ">";
    blemidi_register_console_commands();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
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

    setup_console();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}