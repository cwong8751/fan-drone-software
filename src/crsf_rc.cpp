#include "crsf_rc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "config.h"

static uint8_t rx_buffer[128];
static int16_t channels[16];
bool crsf_initialized = false;

void crsf_init()
{
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
    };

    esp_err_t ret = uart_param_config(CRSF_UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE("CRSF", "uart_param_config failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_set_pin(CRSF_UART_NUM, CRSF_TX_PIN, CRSF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE("CRSF", "uart_set_pin failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_driver_install(CRSF_UART_NUM, sizeof(rx_buffer) * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE("CRSF", "uart_driver_install failed: %s", esp_err_to_name(ret));
        return;
    }

    crsf_initialized = true;
    ESP_LOGI("CRSF", "CRSF UART initialized at %d buad", CRSF_BAUD_RATE);
}

void crsf_update()
{
    if (!crsf_initialized)
    {
        ESP_LOGW("CRSF", "crsf not initialized, skipping read.");
        return;
    }
    
    int len = uart_read_bytes(CRSF_UART_NUM, rx_buffer, sizeof(rx_buffer), 10 / portTICK_PERIOD_MS);
    if (len <= 0) return;

    // search for valid CRSF frame
    for (int i = 0; i < len - 2; i++)
    {
        if (rx_buffer[i] == 0xC8) // CRSF SYNC BYTE
        {
            uint8_t frame_len = rx_buffer[i + 1]; // TODO: need to implement checksum safety to ensure entact data frames
            if (i + frame_len + 2 > len) continue;

            uint8_t type = rx_buffer[i + 2];
            if (type == 0x16) // rc channel data frame
            {
                uint8_t *payload = &rx_buffer[i + 3];
                uint8_t ch_idx = 0;
                uint8_t bit_buffer = 0;
                uint8_t bits_in_buffer = 0;

                for (int j = 0; j < 22; j++)
                {
                    bit_buffer |= ((uint32_t)payload[j]) << bits_in_buffer;
                    bits_in_buffer += 8;

                    while (bits_in_buffer >= 11 && ch_idx < 16)
                    {
                        channels[ch_idx++] = bit_buffer & 0x7FF;
                        bit_buffer >>= 11;
                        bits_in_buffer -= 11;
                    }
                }
            }
        }
    }
}

int16_t crsf_get_channel(uint8_t idx)
{
    if (idx >= 16) return 0;
    return channels[idx];
}