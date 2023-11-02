#include "esp_log.h"
#include "bmx280.h"
#include <math.h>
#include "cmd.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "BNO055ESP32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DShotRMT.h"

static const char *TAG = "QUAD MAIN";

// ----- config for gyro BNO055 -----
#define GYRO_TXD_PIN (GPIO_NUM_17)
#define GYRO_RXD_PIN (GPIO_NUM_16)
static const int RX_BUF_SIZE = 1024;
int num = 0;
// ----- end gyro -----

// ----- config for GPS -----
#define GPS_TXD_PIN (GPIO_NUM_10)
#define GPS_RXD_PIN (GPIO_NUM_9)
// ----- end GPS -----

// ----- config for dshot -----
#define DSHOT_RMT_CHANNEL1 RMT_CHANNEL_0
#define DSHOT_RMT_CHANNEL2 RMT_CHANNEL_1
#define DSHOT_RMT_CHANNEL3 RMT_CHANNEL_2
#define DSHOT_RMT_CHANNEL4 RMT_CHANNEL_3
#define DSHOT_GPIO1 GPIO_NUM_25
#define DSHOT_GPIO2 GPIO_NUM_26
#define DSHOT_GPIO3 GPIO_NUM_27
#define DSHOT_GPIO4 GPIO_NUM_14

#define MIN_THROTTLE 100
#define FULL_THROTTLE 2047

DShotRMT esc1;
DShotRMT esc2;
DShotRMT esc3;
DShotRMT esc4;
// ---- end dshot ----

// ---- config for pressure sensor -----
/* Create Queue */
QueueHandle_t xQueueCmd;
QueueHandle_t xQueueSend;

void init_gps_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPS_TXD_PIN, GPS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void gps_rx_task(void *arg)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(TAG, "Read %d bytes: '%s'", rxBytes, data);
        }
    }
    free(data);
}

static bno055_offsets_t getGyroStoredOffsets()
{
    bno055_offsets_t storedOffsets;
    storedOffsets.accelOffsetX = -1;
    storedOffsets.accelOffsetY = -26;
    storedOffsets.accelOffsetZ = -24;
    storedOffsets.magOffsetX = 60;
    storedOffsets.magOffsetY = -103;
    storedOffsets.magOffsetZ = -882;
    storedOffsets.gyroOffsetX = -2;
    storedOffsets.gyroOffsetY = -3;
    storedOffsets.gyroOffsetZ = 5;
    storedOffsets.accelRadius = 1000;
    storedOffsets.magRadius = 469;
    return storedOffsets;
}

static void rampThrottle(int start, int stop, int step)
{
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step)
    {
        esc1.sendThrottle(i);
        esc2.sendThrottle(i);
        esc3.sendThrottle(i);
        esc4.sendThrottle(i);
        vTaskDelay(10);
    }
    esc1.sendThrottle(stop);
    esc2.sendThrottle(stop);
    esc3.sendThrottle(stop);
    esc4.sendThrottle(stop);
}

static void sendGeneralThrottle(int throttle)
{
    esc1.sendThrottle(throttle);
    esc2.sendThrottle(throttle);
    esc3.sendThrottle(throttle);
    esc4.sendThrottle(throttle);
}

static void startESCs()
{
    // init need to send 0 throttle for a while to start up the esc
    const TickType_t holdStop = xTaskGetTickCount() + (5000 / portTICK_PERIOD_MS);
    while (xTaskGetTickCount() < holdStop)
    {
        esc1.sendThrottle(0);
        esc2.sendThrottle(0);
        esc3.sendThrottle(0);
        esc4.sendThrottle(0);
        vTaskDelay(1);
    }
}

extern "C" void app_main(void)
{
    vTaskDelay(200); // Wait for a bit for the modules to power up

    // Initialize NVS
     ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init Gyro
     ESP_LOGI(TAG, "Initializing Gyro");
    bno055_offsets_t storedOffsets = getGyroStoredOffsets();

    // Init GPS
     ESP_LOGI(TAG, "Initializing GPS");
    init_gps_uart();
    xTaskCreate(gps_rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);

    // Init Pressure sensor
     ESP_LOGI(TAG, "Initializing Pressure sensor");
    xQueueCmd = xQueueCreate(10, sizeof(CMD_t));
    configASSERT(xQueueCmd);
    xQueueSend = xQueueCreate(10, sizeof(CMD_t));
    configASSERT(xQueueSend);
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = false,
        .scl_pullup_en = false,

        .master = {
            .clk_speed = 100000}};

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    bmx280_t *bmx280 = bmx280_create(I2C_NUM_0);
    if (!bmx280)
    {
        ESP_LOGE(TAG, "Could not create bmx280 driver.");
        return;
    }
    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));
    CMD_t cmdBuf;

    // Init BNO055
    BNO055BaseException ex;

    BNO055 bno(UART_NUM_2, GYRO_TXD_PIN, GYRO_RXD_PIN);
    bno.begin(); // BNO055 is in CONFIG_MODE until it is changed
    bno.enableExternalCrystal();
    bno.setSensorOffsets(storedOffsets);
    bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
    bno.setOprModeNdof();
    ESP_LOGI(TAG, "BNO055 Setup Done.");

    // Init Dshot protocol
    ESP_LOGI(TAG, "Initializing DShot RMT");
    ESP_ERROR_CHECK(esc1.install(DSHOT_GPIO1, DSHOT_RMT_CHANNEL1));
    ESP_ERROR_CHECK(esc2.install(DSHOT_GPIO2, DSHOT_RMT_CHANNEL2));
    ESP_ERROR_CHECK(esc3.install(DSHOT_GPIO3, DSHOT_RMT_CHANNEL3));
    ESP_ERROR_CHECK(esc4.install(DSHOT_GPIO4, DSHOT_RMT_CHANNEL4));

    startESCs();

    rampThrottle(MIN_THROTTLE, 400, 10);

    while (1)
    {
        //xQueueReceive(xQueueCmd, &cmdBuf, 1);
        float temp = 0, pres = 0, hum = 0, hpa = 0;
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));
        hpa = pres / 100;
        float p0 = 1013.25f;
        float altitude = (float)(44330 * (1 - pow((hpa / p0), (1 / 5.255))));
        ESP_LOGI(TAG, "Read Values: temp = %f, pres = %f, hum = %f, alt = %f", temp, pres, hum, altitude);
        bno055_vector_t v = bno.getVectorEuler();
        ESP_LOGI(TAG, "Euler: X: %.2f Y: %.2f Z: %.2f", v.x, v.y, v.z);
        sendGeneralThrottle(400);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}