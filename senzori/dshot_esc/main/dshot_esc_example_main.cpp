#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

#define DSHOT_ESC_GPIO_NUM_MOTOR1 GPIO_NUM_25
#define DSHOT_ESC_GPIO_NUM_MOTOR2 GPIO_NUM_26

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000
#endif

static const char *TAG = "example";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Create RMT TX channels");

    // Configure RMT TX channel for Motor 1
    rmt_channel_handle_t esc_chan_motor1 = NULL;
    rmt_tx_channel_config_t tx_chan_config_motor1 = {
        .gpio_num = DSHOT_ESC_GPIO_NUM_MOTOR1,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 10,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_motor1, &esc_chan_motor1));

    // Configure RMT TX channel for Motor 2
    rmt_channel_handle_t esc_chan_motor2 = NULL;
    rmt_tx_channel_config_t tx_chan_config_motor2 = {
        .gpio_num = DSHOT_ESC_GPIO_NUM_MOTOR2,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 10,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_motor2, &esc_chan_motor2));

    ESP_LOGI(TAG, "Install Dshot ESC encoders");

    // Configure Dshot ESC encoder for Motor 1
    rmt_encoder_handle_t dshot_encoder_motor1 = nullptr;
    dshot_esc_encoder_config_t encoder_config_motor1 = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000,
        .post_delay_us = 50,
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config_motor1, &dshot_encoder_motor1));

    // Configure Dshot ESC encoder for Motor 2
    rmt_encoder_handle_t dshot_encoder_motor2 = nullptr;
    dshot_esc_encoder_config_t encoder_config_motor2 = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000,
        .post_delay_us = 50,
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config_motor2, &dshot_encoder_motor2));

    ESP_LOGI(TAG, "Enable RMT TX channels");
    ESP_ERROR_CHECK(rmt_enable(esc_chan_motor1));
    ESP_ERROR_CHECK(rmt_enable(esc_chan_motor2));

    rmt_transmit_config_t tx_config = {
        .loop_count = -1,
        .flags = 0,
    };

    dshot_esc_throttle_t throttle_motor1 = {
        .throttle = 0,
        .telemetry_req = false,
    };

    dshot_esc_throttle_t throttle_motor2 = {
        .throttle = 0,
        .telemetry_req = false,
    };

    ESP_LOGI(TAG, "Start ESCs by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan_motor1, dshot_encoder_motor1, &throttle_motor1, sizeof(throttle_motor1), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_chan_motor2, dshot_encoder_motor2, &throttle_motor2, sizeof(throttle_motor2), &tx_config));

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Increase throttle, no telemetry");
    for (uint16_t thro = 100; thro < 1000; thro += 10) {
        throttle_motor1.throttle = thro;
        throttle_motor2.throttle = thro;

        ESP_ERROR_CHECK(rmt_transmit(esc_chan_motor1, dshot_encoder_motor1, &throttle_motor1, sizeof(throttle_motor1), &tx_config));
        ESP_ERROR_CHECK(rmt_transmit(esc_chan_motor2, dshot_encoder_motor2, &throttle_motor2, sizeof(throttle_motor2), &tx_config));

        // Stop and restart channels to update the new throttle
        ESP_ERROR_CHECK(rmt_disable(esc_chan_motor1));
        ESP_ERROR_CHECK(rmt_disable(esc_chan_motor2));

        ESP_ERROR_CHECK(rmt_enable(esc_chan_motor1));
        ESP_ERROR_CHECK(rmt_enable(esc_chan_motor2));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
