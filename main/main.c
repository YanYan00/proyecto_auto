#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "SparkFunLSM9DS1.h"

#define I2C_MASTER_SDA_IO           23
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void) 
{
    printf("\n=== INICIALIZANDO SENSOR LSM9DS1 (9DOF) ===\n");

    if (i2c_master_init() != ESP_OK) {
        fprintf(stderr, "Error al configurar el puerto I2C\n");
        return;
    }

    // Inicializamos con parámetro 1 para usar las direcciones 0x6B y 0x1E
    uint16_t status = lsm9ds1_begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), I2C_MASTER_NUM);
    
    if (status == 0) {
        fprintf(stderr, "Error al inicializar los registros del LSM9DS1.\n");
        return;
    }

    printf("¡Sensor LSM9DS1 listo! ID Detectado: 0x%04X\n\n", status);

    // Ajustamos escalas de sensibilidad
    lsm9ds1_setAccelScale(8);     // +/- 8g
    lsm9ds1_setGyroScale(245);    // +/- 245 dps
    lsm9ds1_setMagScale(4);       // +/- 4 Gauss

    while (1) {
        int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
        int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;
        int16_t raw_mx = 0, raw_my = 0, raw_mz = 0;

        // 1. Leer datos crudos de los 9 ejes
        lsm9ds1_readAccel(&raw_ax, &raw_ay, &raw_az);
        lsm9ds1_readGyro(&raw_gx, &raw_gy, &raw_gz);
        lsm9ds1_readMag(&raw_mx, &raw_my, &raw_mz);

        // 2. Convertir a unidades físicas (g, dps, Gauss)
        float ax = lsm9ds1_calcAccel(raw_ax);
        float ay = lsm9ds1_calcAccel(raw_ay);
        float az = lsm9ds1_calcAccel(raw_az);

        float gx = lsm9ds1_calcGyro(raw_gx);
        float gy = lsm9ds1_calcGyro(raw_gy);
        float gz = lsm9ds1_calcGyro(raw_gz);

        float mx = lsm9ds1_calcMag(raw_mx);
        float my = lsm9ds1_calcMag(raw_my);
        float mz = lsm9ds1_calcMag(raw_mz);

        // 3. Imprimir mediciones
        printf("[ACCEL g]  X: %6.2f | Y: %6.2f | Z: %6.2f\n", ax, ay, az);
        printf("[GYRO dps] X: %6.2f | Y: %6.2f | Z: %6.2f\n", gx, gy, gz);
        printf("[MAG  Ga]  X: %6.2f | Y: %6.2f | Z: %6.2f\n", mx, my, mz);
        printf("---------------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}