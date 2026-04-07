/*
 * MAX30102 Simple Heart Rate Monitor with Fall Detection
 * Simplified for continuous, reliable readings
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
// #include <src/sensors/mpu6050.h>

/* MAX30102 I2C Address */
#define MAX30102_ADDR 0x57

/* MAX30102 Register Addresses */
#define MAX30102_FIFO_WR_PTR     0x04
#define MAX30102_FIFO_RD_PTR     0x06
#define MAX30102_FIFO_DATA       0x07
#define MAX30102_FIFO_CONFIG     0x08
#define MAX30102_MODE_CONFIG     0x09
#define MAX30102_SPO2_CONFIG     0x0A
#define MAX30102_LED1_PA         0x0C
#define MAX30102_LED2_PA         0x0D
#define MAX30102_REV_ID          0xFE
#define MAX30102_PART_ID         0xFF

/* LED definitions */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* I2C device */
static const struct device *i2c_dev;

/* Calibration baselines */
static uint32_t baseline_red = 0;
static uint32_t baseline_ir = 0;
static bool calibrated = false;

/* Finger detection threshold */
#define FINGER_THRESHOLD 10000

/* Fall detection threshold */
#define FALL_DETECTION_HR_THRESHOLD 90

/* Number of samples for baseline */
#define BASELINE_SAMPLES 50

/* Simple circular buffer for IR values */
#define BUFFER_SIZE 150  /* 1.5 seconds at 100Hz */
static int32_t ir_buffer[BUFFER_SIZE];
static int buffer_index = 0;
static bool buffer_full = false;

/* Heart rate smoothing */
#define HR_HISTORY_SIZE 5
static int32_t hr_history[HR_HISTORY_SIZE] = {0};
static int hr_history_index = 0;
static int hr_history_count = 0;

/* Function to write a register */
int max30102_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_write(i2c_dev, data, 2, MAX30102_ADDR);
}

/* Function to read a register */
int max30102_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_write_read(i2c_dev, MAX30102_ADDR, &reg, 1, value, 1);
}

/* Initialize MAX30102 sensor */
int max30102_init(void)
{
    uint8_t part_id;
    int ret;

    ret = max30102_read_reg(MAX30102_PART_ID, &part_id);
    if (ret < 0) {
        printk("Failed to read Part ID\n");
        return ret;
    }
    printk("MAX30102 Part ID: 0x%02X\n", part_id);

    /* Reset the sensor */
    max30102_write_reg(MAX30102_MODE_CONFIG, 0x40);
    k_msleep(100);

    /* Configure FIFO */
    max30102_write_reg(MAX30102_FIFO_CONFIG, 0x4F);

    /* Set Mode: SpO2 mode */
    max30102_write_reg(MAX30102_MODE_CONFIG, 0x03);

    /* Configure SpO2: 100Hz sample rate */
    max30102_write_reg(MAX30102_SPO2_CONFIG, 0x27);

    /* Set LED pulse amplitude */
    max30102_write_reg(MAX30102_LED1_PA, 0x24);
    max30102_write_reg(MAX30102_LED2_PA, 0x24);

    printk("MAX30102 initialized successfully!\n");
    return 0;
}

/* Read FIFO data */
int max30102_read_fifo(uint32_t *red_led, uint32_t *ir_led)
{
    uint8_t fifo_data[6];
    uint8_t reg = MAX30102_FIFO_DATA;
    int ret;

    ret = i2c_write_read(i2c_dev, MAX30102_ADDR, &reg, 1, fifo_data, 6);
    if (ret < 0) {
        return ret;
    }

    *red_led = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
    *ir_led = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];

    *red_led &= 0x3FFFF;
    *ir_led &= 0x3FFFF;

    return 0;
}

/* Perform baseline calibration */
void calibrate_baseline(void)
{
    uint32_t sum_red = 0;
    uint32_t sum_ir = 0;
    uint32_t red, ir;
    int valid_samples = 0;

    printk("\nCalibrating... (keep finger OFF)\n");
    k_msleep(2000);

    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        if (max30102_read_fifo(&red, &ir) == 0) {
            sum_red += red;
            sum_ir += ir;
            valid_samples++;
        }
        k_msleep(10);
    }

    if (valid_samples > 0) {
        baseline_red = sum_red / valid_samples;
        baseline_ir = sum_ir / valid_samples;
        calibrated = true;
        printk("Calibration done! Baseline IR: %u\n\n", baseline_ir);
    }
}

/* Simple autocorrelation-based heart rate detection */
int32_t calculate_heart_rate_simple(void)
{
    if (!buffer_full) {
        return 0;
    }

    /* Calculate mean */
    int64_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += ir_buffer[i];
    }
    int32_t mean = sum / BUFFER_SIZE;

    /* Find the strongest peak in autocorrelation between 0.5s and 2s */
    /* This corresponds to 30-120 BPM */
    int min_lag = 50;   /* 0.5 seconds = 120 BPM */
    int max_lag = 120;  /* 1.2 seconds = 50 BPM */
    
    int64_t max_correlation = 0;
    int best_lag = 0;

    for (int lag = min_lag; lag < max_lag; lag++) {
        int64_t correlation = 0;
        for (int i = 0; i < BUFFER_SIZE - lag; i++) {
            int32_t val1 = ir_buffer[i] - mean;
            int32_t val2 = ir_buffer[i + lag] - mean;
            correlation += (int64_t)val1 * val2;
        }
        
        if (correlation > max_correlation) {
            max_correlation = correlation;
            best_lag = lag;
        }
    }

    /* Convert lag to BPM */
    if (best_lag > 0 && max_correlation > 0) {
        /* BPM = (60 seconds * 100 samples/sec) / lag */
        return (60 * 100) / best_lag;
    }

    return 0;
}

/* Add heart rate to history and return smoothed value */
int32_t smooth_heart_rate(int32_t new_hr)
{
    if (new_hr == 0) {
        return 0;
    }

    /* Add to history */
    hr_history[hr_history_index] = new_hr;
    hr_history_index = (hr_history_index + 1) % HR_HISTORY_SIZE;
    
    if (hr_history_count < HR_HISTORY_SIZE) {
        hr_history_count++;
    }

    /* Calculate average */
    int64_t sum = 0;
    for (int i = 0; i < hr_history_count; i++) {
        sum += hr_history[i];
    }

    return sum / hr_history_count;
}

int max_30102(void)
{
    int ret;
    uint32_t red_led, ir_led;
    int32_t red_calibrated, ir_calibrated;
    int led_state = 0;
    int32_t heart_rate_raw = 0;
    int32_t heart_rate_smooth = 0;

    printk("\n========================================\n");
    printk("Simple Heart Rate Monitor\n");
    printk("Fall Detection: HR > %d bpm\n", FALL_DETECTION_HR_THRESHOLD);
    printk("========================================\n\n");

    /* Configure LED */
    if (!gpio_is_ready_dt(&led)) {
        printk("LED device not ready\n");
        return 0;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    /* Get I2C device */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return 0;
    }
    printk("I2C device ready\n");

    /* Initialize MAX30102 */
    ret = max30102_init();
    if (ret < 0) {
        printk("Failed to initialize MAX30102\n");
        return 0;
    }

    /* Perform baseline calibration */
    calibrate_baseline();

    printk("Place finger on sensor...\n\n");

    /* Calculation counter */
    int calc_counter = 0;

    while (1) {
        gpio_pin_set_dt(&led, led_state);
        led_state = !led_state;

        /* Read sensor data */
        ret = max30102_read_fifo(&red_led, &ir_led);
        if (ret == 0 && calibrated) {
            /* Apply baseline calibration */
            red_calibrated = (int32_t)red_led - (int32_t)baseline_red;
            ir_calibrated = (int32_t)ir_led - (int32_t)baseline_ir;
            
            if (red_calibrated < 0) red_calibrated = 0;
            if (ir_calibrated < 0) ir_calibrated = 0;

            /* Store in buffer */
            ir_buffer[buffer_index] = ir_calibrated;
            buffer_index++;
            if (buffer_index >= BUFFER_SIZE) {
                buffer_index = 0;
                buffer_full = true;
            }

            /* Calculate heart rate every 50 samples (0.5 seconds) */
            calc_counter++;
            if (calc_counter >= 50) {
                calc_counter = 0;
                
                if (ir_calibrated > FINGER_THRESHOLD) {
                    heart_rate_raw = calculate_heart_rate_simple();
                    
                    if (heart_rate_raw > 0) {
                        heart_rate_smooth = smooth_heart_rate(heart_rate_raw);
                    }
                } else {
                    heart_rate_raw = 0;
                    heart_rate_smooth = 0;
                    hr_history_count = 0;
                    buffer_full = false;
                    buffer_index = 0;
                }
            }

            /* Display every 100 samples (1 second) */
            static int display_counter = 0;
            if (++display_counter >= 100) {
                display_counter = 0;
                
                if (ir_calibrated > FINGER_THRESHOLD) {
                    if (heart_rate_smooth > 0 && hr_history_count >= 3) {
                        printk("HR: %3d bpm", heart_rate_smooth);
                        
                        /* Fall detection */
                        if (heart_rate_smooth > FALL_DETECTION_HR_THRESHOLD) {
                            printk(" | *** FALL ALERT! ***");
                        } else {
                            printk(" | NORMAL");
                        }
                        printk("\n");
                    } else {
                        printk("Reading... (wait 3s)\n");
                    }
                } else {
                    printk("No finger detected\n");
                }
            }
        }

        k_msleep(10);  /* 100 Hz sampling */
    }

    return 0;
}