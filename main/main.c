#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_log.h"
#include "linenoise/linenoise.h"
#include "blemidi.h"

#define TAG "ble-midi-cv"
#define PROMPT_STR CONFIG_IDF_TARGET

#define GATE_PIN 27

#define DAC_I2C_SDA 21
#define DAC_I2C_SCL 22
#define DAC_I2C_FREQ 400000
#define DAC_I2C_ADDR 0x60
#define DAC_VMAX 3.3f

#define NN_V0 36 // C2

static uint8_t last_note;
static bool note_on = false;
static uint16_t t_last = 0;

uint16_t nn_to_dac_value(uint8_t nn)
{
    uint8_t nn_scaled = (nn < NN_V0 ? NN_V0 : nn) - NN_V0;
    float oct_mult = (float)(nn_scaled / (12.0f * DAC_VMAX));
    int value = (int)(oct_mult * 4095.0f);
    ESP_LOGD(TAG, "Raw nn:%d, Scaled nn:%d, DAC value: %d", nn, nn_scaled, value);
    return value > 4095 ? 4095 : value;
}

void gpio_gate_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GATE_PIN);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

esp_err_t i2c_dac_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DAC_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = DAC_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = DAC_I2C_FREQ
    };
    esp_err_t err;
    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_param_config(I2C_NUM_0, &conf);
}

esp_err_t i2c_dac_write(uint16_t value)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return err;
    }
    i2c_master_write_byte(cmd, DAC_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    uint8_t data[3] = {0x40, value >> 4, value << 4};
    i2c_master_write(cmd, data, 3, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return err;
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

void callback_midi_message_received(uint8_t blemidi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
    ESP_LOGI(TAG, "CALLBACK blemidi_port=%d, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", blemidi_port, timestamp, midi_status, len, continued_sysex_pos);

    uint8_t code = midi_status >> 4;
    // uint8_t channel = midi_status & 0x0F;
    if (code == 0x09) // note on, ignore channel
    {
        uint16_t t = timestamp;
        if (t < t_last) t += (1 << 13);
        ESP_LOGI(TAG, "Delta t: %d", t - t_last);
        t_last = timestamp;

        last_note = remaining_message[0];
        if (note_on)
        {
            // gate off, update CV, gate on
            gpio_set_level(GATE_PIN, 0);
            i2c_dac_write(nn_to_dac_value(last_note));
            // vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(GATE_PIN, 1);
        }
        else
        {
            // update CV, gate on
            i2c_dac_write(nn_to_dac_value(last_note));
            gpio_set_level(GATE_PIN, 1);
        }
        note_on = true;
    }
    else if (code == 0x08) // note off
    {
        // Only stop gate if same note off
        if (last_note == remaining_message[0])
        {
            gpio_set_level(GATE_PIN, 0);
            note_on = false;
        }
    }
}

void app_main(void)
{
//    setup_console();

    int status = blemidi_init(callback_midi_message_received);
    if (status < 0)
    {
        ESP_LOGE(TAG, "BLE MIDI Driver returned status=%d", status);
    }
    else
    {
        ESP_LOGI(TAG, "BLE MIDI Driver initialized successfully");
    }

    gpio_gate_init();
    ESP_ERROR_CHECK(i2c_dac_init());
}