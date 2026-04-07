#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ================= MPU6050 DEFINITIONS ================= */

#define MPU6050_ADDR        0x68

#define WHO_AM_I            0x75
#define PWR_MGMT_1          0x6B

#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43

#define MPU6050_WHOAMI_VAL  0x68

#define I2C_NODE DT_NODELABEL(i2c1)

/* ================= GLOBALS ================= */

static const struct device *i2c_dev;

/* ================= I2C HELPERS ================= */

static int mpu6050_write(uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(i2c_dev, MPU6050_ADDR, reg, val);
}

static int mpu6050_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_burst_read(i2c_dev, MPU6050_ADDR, reg, buf, len);
}

/* ================= MPU6050 INIT ================= */

static int mpu6050_init(void)
{
    uint8_t whoami = 0;

    if (mpu6050_read(WHO_AM_I, &whoami, 1)) {
        printk("WHO_AM_I read failed\n");
        return -1;
    }

    printk("MPU6050 WHO_AM_I = 0x%x\n", whoami);

    if (whoami != MPU6050_WHOAMI_VAL) {
        printk("MPU6050 not detected\n");
        return -1;
    }

    if (mpu6050_write(PWR_MGMT_1, 0x00)) {
        printk("Failed to wake MPU6050\n");
        return -1;
    }

    printk("MPU6050 initialized\n");
    return 0;
}

/* ================= SENSOR READ ================= */

static int read_accel_gyro(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[14];

    if (mpu6050_read(ACCEL_XOUT_H, data, 14)) {
        return -1;
    }

    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);

    *gx = (int16_t)((data[8] << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);

    return 0;
}

/* ================= FALL DETECTION ================= */

/*
 * DEMO-FRIENDLY LOGIC:
 *
 * 1) Free-fall (low accel)
 * 2) Impact (high accel)
 * 3) Orientation change OR gyro spike
 *
 * Accel scale: ±2g → 16384 LSB/g
 * Gyro scale : ±250 dps → 131 LSB/dps
 */

static bool detect_fall(int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz)
{
    static bool free_fall = false;
    static int64_t last_fall_time = 0;

    int64_t now = k_uptime_get();

    /* ---------- Accel magnitude ---------- */
    int64_t acc_mag_sq =
        (int64_t)ax * ax +
        (int64_t)ay * ay +
        (int64_t)az * az;

    /* ---------- Gyro magnitude ---------- */
    int32_t gyro_mag =
        abs(gx) + abs(gy) + abs(gz);

    /* ---------- Orientation (tilt angle) ---------- */
    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float tilt_angle =
        atan2f(sqrtf(ax_g * ax_g + ay_g * ay_g), az_g) * 57.3f;

    /* ---------- DEMO THRESHOLDS ---------- */
    #define FREE_FALL_TH        (7000LL * 7000LL)    /* ~0.43g */
    #define IMPACT_TH           (18000LL * 18000LL) /* ~1.1g */
    #define GYRO_SPIKE_TH       2000                /* raw sum */
    #define TILT_ANGLE_TH       45.0f               /* degrees */
    #define FALL_COOLDOWN_MS    3000

    if (now - last_fall_time < FALL_COOLDOWN_MS) {
        return false;
    }

    printk("AccMag^2=%lld Gyro=%d Tilt=%.1f\n",
           acc_mag_sq, gyro_mag, tilt_angle);

    /* ---------- Stage 1: Free fall ---------- */
    if (acc_mag_sq < FREE_FALL_TH) {
        printk("Free-fall detected\n");
        free_fall = true;
    }

    /* ---------- Stage 2: Impact + orientation ---------- */
    if (free_fall &&
        acc_mag_sq > IMPACT_TH &&
        (gyro_mag > GYRO_SPIKE_TH || tilt_angle > TILT_ANGLE_TH)) {

        printk("\n===========================\n");
        printk("   ⚠️  FALL DETECTED ⚠️\n");
        printk("===========================\n\n");

        free_fall = false;
        last_fall_time = now;
        return true;
    }

    return false;
}

/* ================= MAIN ================= */

int mpu6050(void)
{
    printk("\n=== MPU6050 FALL DETECTION (ACCEL + GYRO) ===\n");

    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return 0;
    }

    if (mpu6050_init()) {
        printk("MPU6050 init failed\n");
        return 0;
    }

    while (1) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        if (read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            printk("A:%6d %6d %6d | G:%6d %6d %6d\n",
                   ax, ay, az, gx, gy, gz);

            detect_fall(ax, ay, az, gx, gy, gz);
        }

        k_sleep(K_MSEC(200));
    }
}