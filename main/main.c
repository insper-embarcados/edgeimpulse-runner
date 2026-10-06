/*
 * main.c — Lab Expert DSP/IA
 * MPU6050 (acelerometro) -> Edge Impulse (idle / updown / wave) -> LED RGB
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Funcoes da IA (definidas em ia.cpp) */
extern int ia_classificar(float *features, int n, float *confianca_out, const char **label_out);
extern int ia_window_size(void);

/* LED RGB */
#define LED_IDLE    18
#define LED_UPDOWN  19
#define LED_WAVE    20

/* MPU6050 */
#define I2C_PORT    i2c0
#define I2C_SDA     8
#define I2C_SCL     9
#define MPU_ADDR    0x68

#define IA_WINDOW      166   /* = EI_CLASSIFIER_RAW_SAMPLE_COUNT */
#define IMU_PERIOD_MS  10    /* 100 Hz: deve bater com a freq. da coleta */

typedef struct {
    float data[IA_WINDOW * 3];
} ia_window_t;

QueueHandle_t xQueueIMU;

/* ===== MPU6050 ===== */
static void mpu6050_write(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

static void mpu6050_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDR, buf, len, false);
}

/* ===== Task: le o acelerometro e monta a janela ===== */
static void imu_task(void *params) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    mpu6050_write(0x6B, 0x00);   /* tira o MPU do sleep */

    uint8_t who = 0;
    mpu6050_read(0x75, &who, 1);
    printf("[MPU6050] WHO_AM_I = 0x%02X %s\n", who, (who == 0x68) ? "OK" : "ERRO");

    static ia_window_t win;
    int idx = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        uint8_t a[6];
        mpu6050_read(0x3B, a, 6);
        int16_t ax = (int16_t)((a[0] << 8) | a[1]);
        int16_t ay = (int16_t)((a[2] << 8) | a[3]);
        int16_t az = (int16_t)((a[4] << 8) | a[5]);

        win.data[idx * 3 + 0] = (float)ax;
        win.data[idx * 3 + 1] = (float)ay;
        win.data[idx * 3 + 2] = (float)az;
        idx++;

        if (idx >= IA_WINDOW) {
            idx = 0;
            xQueueSend(xQueueIMU, &win, 0);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/* ===== Task: classifica e acende o LED ===== */
static void ia_task(void *params) {
    gpio_init(LED_IDLE);   gpio_set_dir(LED_IDLE,   GPIO_OUT);
    gpio_init(LED_UPDOWN); gpio_set_dir(LED_UPDOWN, GPIO_OUT);
    gpio_init(LED_WAVE);   gpio_set_dir(LED_WAVE,   GPIO_OUT);

    static ia_window_t win;

    while (true) {
        if (xQueueReceive(xQueueIMU, &win, portMAX_DELAY) != pdTRUE) continue;

        float conf = 0.0f;
        const char *label = "?";
        int gesto = ia_classificar(win.data, IA_WINDOW * 3, &conf, &label);

        if (gesto < 0) {
            printf("[IA] erro\n");
            continue;
        }

        printf("[IA] %s (%.2f)\n", label, (double)conf);

        gpio_put(LED_IDLE,   strcmp(label, "idle")   == 0);
        gpio_put(LED_UPDOWN, strcmp(label, "updown") == 0);
        gpio_put(LED_WAVE,   strcmp(label, "wave")   == 0);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== Lab Expert: IA de movimento ===\n");
    printf("[IA] janela: %d amostras x 3 eixos\n", ia_window_size());

    xQueueIMU = xQueueCreate(2, sizeof(ia_window_t));

    xTaskCreate(imu_task, "imu", 1024, NULL, 1, NULL);
    xTaskCreate(ia_task,  "ia",  4096, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true) {}
    return 0;
}