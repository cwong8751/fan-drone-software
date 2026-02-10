#include "crsf_rc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "config.h"
#include <Arduino.h>

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
        //Serial.println("CRSF UART param config failed");
        return;
    }

    ret = uart_set_pin(CRSF_UART_NUM, CRSF_TX_PIN, CRSF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE("CRSF", "uart_set_pin failed: %s", esp_err_to_name(ret));
        //Serial.println("CRSF UART set pin failed");
        return;
    }

    ret = uart_driver_install(CRSF_UART_NUM, sizeof(rx_buffer) * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE("CRSF", "uart_driver_install failed: %s", esp_err_to_name(ret));
        //Serial.println("CRSF UART driver install failed");
        return;
    }

    crsf_initialized = true;
    ESP_LOGI("CRSF", "CRSF UART initialized at %d buad", CRSF_BAUD_RATE);
    Serial.printf("CRSF UART initialized at %d baud\n", CRSF_BAUD_RATE);
}

void crsf_update()
{
    if (!crsf_initialized)
    {
        ESP_LOGW("CRSF", "crsf not initialized, skipping read.");
        Serial.println("CRSF not initialized, skipping read.");
        return;
    }
    
    int len = uart_read_bytes(CRSF_UART_NUM, rx_buffer, sizeof(rx_buffer), 10 / portTICK_PERIOD_MS);
    //Serial.printf("length of bytes: %d\n", len);
    if (len <= 0) return;

    // search for valid CRSF frame
    for (int i = 0; i < len - 2; i++)
    {
        if (rx_buffer[i] == 0xC8) // CRSF SYNC BYTE
        {
            uint8_t frame_len = rx_buffer[i + 1]; // TODO: need to implement checksum safety to ensure entact data frames
            if (i + frame_len + 2 > len) continue;

            uint8_t type = rx_buffer[i + 2];
            //Serial.printf("Frame: SYNC 0xC8, len=%d, type=0x%02X\n", len, type);
            if (type == 0x16) // RC channels frame
            {
                uint8_t *payload = &rx_buffer[i + 3]; // start of channel data

                uint32_t bitBuffer = 0;
                uint8_t bitsInBuffer = 0;
                uint8_t targetChannel = 2;  // <-- channel 3 (zero-indexed)
                uint16_t value = 0;
                uint8_t chIdx = 0;

                for (int j = 0; j < 22; j++)  // 22 bytes total payload
                {
                    bitBuffer |= ((uint32_t)payload[j]) << bitsInBuffer;
                    bitsInBuffer += 8;

                    while (bitsInBuffer >= 11)
                    {
                        value = bitBuffer & 0x7FF; // extract 11 bits
                        channels[chIdx] = value;
                        chIdx++;
                        bitBuffer >>= 11;
                        bitsInBuffer -= 11;
                    }
                }
            }
        }
    }
}

static void csrf_debug_throttle_raw()
{
    uint8_t buf[64];
    int len = uart_read_bytes(CRSF_UART_NUM, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
    if (len > 0)
    {
        printf("[CRSF RAW BYTES] %d bytes: ", len);
        for (int i = 0; i < len; i++)
        {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    }
}

int16_t crsf_get_channel(uint8_t idx)
{
    if (idx >= 16) return 0;
    return channels[idx];
}