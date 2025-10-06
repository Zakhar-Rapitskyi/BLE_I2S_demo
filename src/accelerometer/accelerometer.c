/**
 * @file accelerometer.c
 * @brief BMA280 accelerometer driver implementation
 */

#include "accelerometer.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "app_error.h"
#include <string.h>
#include "boards.h"


// BMA280 Register Map
#define BMA280_REG_CHIP_ID 0x00      // Should return 0xFB
#define BMA280_REG_ACCD_X_LSB 0x02   // X-axis LSB
#define BMA280_REG_ACCD_X_MSB 0x03   // X-axis MSB
#define BMA280_REG_ACCD_Y_LSB 0x04   // Y-axis LSB
#define BMA280_REG_ACCD_Y_MSB 0x05   // Y-axis MSB
#define BMA280_REG_ACCD_Z_LSB 0x06   // Z-axis LSB
#define BMA280_REG_ACCD_Z_MSB 0x07   // Z-axis MSB
#define BMA280_REG_INT_STATUS_0 0x09 // Interrupt status
#define BMA280_REG_INT_STATUS_1 0x0A
#define BMA280_REG_INT_EN_0 0x16 // Interrupt enable register
#define BMA280_REG_INT_EN_1 0x17
#define BMA280_REG_INT_MAP_0 0x19 // Map interrupt to INT1 pin
#define BMA280_REG_INT_MAP_1 0x1A
#define BMA280_REG_INT_SRC 0x1E      // Interrupt source
#define BMA280_REG_INT_OUT_CTRL 0x20 // Interrupt output control
#define BMA280_REG_INT_LATCH 0x21    // Interrupt latch mode
#define BMA280_REG_INT_0 0x22        // Slope interrupt config
#define BMA280_REG_INT_1 0x23
#define BMA280_REG_INT_2 0x24
#define BMA280_REG_INT_3 0x25
#define BMA280_REG_INT_4 0x26
#define BMA280_REG_INT_5 0x27         // Slope interrupt threshold
#define BMA280_REG_INT_6 0x28         // Slope interrupt duration
#define BMA280_REG_PMU_RANGE 0x0F     // G-range selection
#define BMA280_REG_PMU_BW 0x10        // Bandwidth selection
#define BMA280_REG_PMU_LPW 0x11       // Power mode
#define BMA280_REG_PMU_LOW_POWER 0x12 // Low power config
#define BMA280_REG_CMD 0x14           // Command register

// Expected CHIP_ID
#define BMA280_CHIP_ID 0xFB

// Power modes
#define BMA280_MODE_NORMAL 0x00
#define BMA280_MODE_DEEP_SUSPEND 0x20
#define BMA280_MODE_LOW_POWER 0x40
#define BMA280_MODE_SUSPEND 0x80

// Range settings (PMU_RANGE register)
#define BMA280_RANGE_2G 0x03  // ±2g
#define BMA280_RANGE_4G 0x05  // ±4g
#define BMA280_RANGE_8G 0x08  // ±8g
#define BMA280_RANGE_16G 0x0C // ±16g

// Bandwidth settings (PMU_BW register)
#define BMA280_BW_7_81HZ 0x08
#define BMA280_BW_15_63HZ 0x09
#define BMA280_BW_31_25HZ 0x0A
#define BMA280_BW_62_5HZ 0x0B
#define BMA280_BW_125HZ 0x0C
#define BMA280_BW_250HZ 0x0D
#define BMA280_BW_500HZ 0x0E
#define BMA280_BW_1000HZ 0x0F

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

typedef struct
{
    uint8_t i2c_address;
    uint8_t int_pin;
    uint8_t range_mg_per_lsb; // Sensitivity: mg per LSB
    volatile bool motion_detected;
    accelerometer_motion_handler_t motion_handler;
    accelerometer_data_t cached_data;
    bool initialized;
} accelerometer_t;

static accelerometer_t accelerometer_state = {0};

/**
 * @brief Write single register
 */
static uint32_t write_register(uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return nrf_drv_twi_tx(&m_twi, accelerometer_state.i2c_address, data, 2, false);
}

/**
 * @brief Read single register
 */
static uint32_t read_register(uint8_t reg, uint8_t *p_value)
{
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, accelerometer_state.i2c_address, &reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return nrf_drv_twi_rx(&m_twi, accelerometer_state.i2c_address, p_value, 1);
}

/**
 * @brief Read multiple registers
 */
static uint32_t read_registers(uint8_t reg, uint8_t *p_buffer, uint8_t length)
{
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, accelerometer_state.i2c_address, &reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return nrf_drv_twi_rx(&m_twi, accelerometer_state.i2c_address, p_buffer, length);
}

/**
 * @brief GPIO interrupt handler - motion detected
 */
static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == accelerometer_state.int_pin)
    {
        accelerometer_state.motion_detected = true;
    }
}

/**
 * @brief Configure BMA280 for motion detection
 */
static uint32_t configure_sensor(void)
{
    ret_code_t err_code;
    uint8_t chip_id;

    // Verify device ID
    err_code = read_register(BMA280_REG_CHIP_ID, &chip_id);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to read CHIP_ID: %d", err_code);
        return err_code;
    }

    if (chip_id != BMA280_CHIP_ID)
    {
        NRF_LOG_ERROR("Wrong CHIP_ID: 0x%02X (expected 0xFB)", chip_id);
        return NRF_ERROR_NOT_FOUND;
    }

    NRF_LOG_INFO("BMA280 detected, CHIP_ID: 0x%02X", chip_id);

    // Soft reset
    err_code = write_register(BMA280_REG_CMD, 0xB6);
    if (err_code != NRF_SUCCESS)
        return err_code;
    nrf_delay_ms(10);

    // Set power mode to NORMAL
    err_code = write_register(BMA280_REG_PMU_LPW, BMA280_MODE_NORMAL);
    if (err_code != NRF_SUCCESS)
        return err_code;
    nrf_delay_ms(1);

    // Set range to ±2g
    err_code = write_register(BMA280_REG_PMU_RANGE, BMA280_RANGE_2G);
    if (err_code != NRF_SUCCESS)
        return err_code;
    accelerometer_state.range_mg_per_lsb = 4; // 2g range = 4 mg/LSB (12-bit resolution)

    // Set bandwidth to 125Hz
    err_code = write_register(BMA280_REG_PMU_BW, BMA280_BW_125HZ);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Configure slope/any-motion interrupt
    // INT_0: Enable X, Y, Z axis for slope detection
    err_code = write_register(BMA280_REG_INT_0, 0x07); // Enable X/Y/Z
    if (err_code != NRF_SUCCESS)
        return err_code;

    // INT_5: Slope threshold (0x14 = ~80mg for 2g range)
    err_code = write_register(BMA280_REG_INT_5, 0x14);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // INT_6: Slope duration (number of samples)
    err_code = write_register(BMA280_REG_INT_6, 0x00); // 1 sample
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Enable slope interrupt
    err_code = write_register(BMA280_REG_INT_EN_0, 0x07); // Enable slope X/Y/Z
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Map slope interrupt to INT1 pin
    err_code = write_register(BMA280_REG_INT_MAP_0, 0x04); // Slope to INT1
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Configure INT1 pin: active high, push-pull
    err_code = write_register(BMA280_REG_INT_OUT_CTRL, 0x0A); // INT1: active high, push-pull
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Latch interrupt until read
    err_code = write_register(BMA280_REG_INT_LATCH, 0x0F); // Latched
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Clear any pending interrupts
    uint8_t int_status;
    read_register(BMA280_REG_INT_STATUS_0, &int_status);

    NRF_LOG_INFO("BMA280 configured for motion detection");
    return NRF_SUCCESS;
}

uint32_t accelerometer_init(const accelerometer_init_t *p_config)
{
    ret_code_t err_code;

    if (p_config == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (accelerometer_state.initialized)
    {
        NRF_LOG_WARNING("Accelerometer already initialized");
        return NRF_SUCCESS;
    }

    // Save config
    accelerometer_state.i2c_address = p_config->i2c_address;
    accelerometer_state.int_pin = 25;
    accelerometer_state.motion_handler = p_config->motion_handler;
    memset(&accelerometer_state.cached_data, 0, sizeof(accelerometer_state.cached_data));

    // Initialize I2C
    nrf_drv_twi_config_t twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    twi_config.scl = ARDUINO_SCL_PIN;
    twi_config.sda = ARDUINO_SDA_PIN;
    twi_config.frequency = NRF_DRV_TWI_FREQ_400K;

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("TWI init failed: %d", err_code);
        return err_code;
    }

    nrf_drv_twi_enable(&m_twi);
    nrf_delay_ms(10);

    // Configure sensor
    err_code = configure_sensor();
    if (err_code != NRF_SUCCESS)
    {
        nrf_drv_twi_uninit(&m_twi);
        return err_code;
    }

    // Setup GPIO interrupt
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("GPIOTE init failed: %d", err_code);
            nrf_drv_twi_uninit(&m_twi);
            return err_code;
        }
    }

    nrf_drv_gpiote_in_config_t gpio_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    gpio_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(accelerometer_state.int_pin, &gpio_config, gpiote_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("GPIOTE in_init failed: %d", err_code);
        nrf_drv_twi_uninit(&m_twi);
        return err_code;
    }

    nrf_drv_gpiote_in_event_enable(accelerometer_state.int_pin, true);

    // Read initial data
    accelerometer_read(&accelerometer_state.cached_data);

    accelerometer_state.initialized = true;
    NRF_LOG_INFO("BMA280 accelerometer initialized");

    return NRF_SUCCESS;
}

uint32_t accelerometer_read(accelerometer_data_t *p_data)
{
    if (!accelerometer_state.initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Read all 6 bytes (X, Y, Z) starting from X_LSB
    uint8_t raw_data[6];
    ret_code_t err_code = read_registers(BMA280_REG_ACCD_X_LSB, raw_data, 6);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to read accelerometer data: %d", err_code);
        return err_code;
    }

    // BMA280 data format: 12-bit left-justified in 16-bit (top 12 bits valid)
    // LSB register bits: [new_data, X3, X2, X1, X0, 0, 0, 0]
    // MSB register bits: [X11, X10, X9, X8, X7, X6, X5, X4]

    // Combine and extract 12-bit values
    int16_t x_raw = (int16_t)(((uint16_t)raw_data[1] << 8) | raw_data[0]);
    int16_t y_raw = (int16_t)(((uint16_t)raw_data[3] << 8) | raw_data[2]);
    int16_t z_raw = (int16_t)(((uint16_t)raw_data[5] << 8) | raw_data[4]);

    // Shift right by 4 to get 12-bit signed value
    x_raw = x_raw >> 4;
    y_raw = y_raw >> 4;
    z_raw = z_raw >> 4;

    // Convert to mg using sensitivity
    p_data->x = x_raw * accelerometer_state.range_mg_per_lsb;
    p_data->y = y_raw * accelerometer_state.range_mg_per_lsb;
    p_data->z = z_raw * accelerometer_state.range_mg_per_lsb;

    NRF_LOG_DEBUG("BMA280: X=%d, Y=%d, Z=%d mg", p_data->x, p_data->y, p_data->z);

    return NRF_SUCCESS;
}

uint32_t accelerometer_get_cached_data(accelerometer_data_t *p_data)
{
    if (p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    *p_data = accelerometer_state.cached_data;
    return NRF_SUCCESS;
}

void accelerometer_data_to_bytes(const accelerometer_data_t *p_data, uint8_t *p_buffer)
{
    if (p_data == NULL || p_buffer == NULL)
    {
        return;
    }

    p_buffer[0] = (uint8_t)(p_data->x & 0xFF);
    p_buffer[1] = (uint8_t)((p_data->x >> 8) & 0xFF);
    p_buffer[2] = (uint8_t)(p_data->y & 0xFF);
    p_buffer[3] = (uint8_t)((p_data->y >> 8) & 0xFF);
    p_buffer[4] = (uint8_t)(p_data->z & 0xFF);
    p_buffer[5] = (uint8_t)((p_data->z >> 8) & 0xFF);
}

uint32_t accelerometer_enable_interrupt(void)
{
    if (!accelerometer_state.initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    nrf_drv_gpiote_in_event_enable(accelerometer_state.int_pin, true);
    return NRF_SUCCESS;
}

uint32_t accelerometer_disable_interrupt(void)
{
    if (!accelerometer_state.initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    nrf_drv_gpiote_in_event_disable(accelerometer_state.int_pin);
    return NRF_SUCCESS;
}

void accelerometer_process_event(void)
{
    if (!accelerometer_state.initialized)
    {
        return;
    }
    
    if(accelerometer_state.motion_detected)
    {
        accelerometer_state.motion_detected = false;
        
        uint8_t int_status;
        ret_code_t err = read_register(BMA280_REG_INT_STATUS_0, &int_status);

        if ((err == NRF_SUCCESS) && (int_status & 0x04))
        {
            accelerometer_data_t data;
            
            err = accelerometer_read(&data);


            if (err == NRF_SUCCESS)
            {
                accelerometer_state.cached_data = data;

                if (accelerometer_state.motion_handler != NULL)
                {
                    accelerometer_state.motion_handler(&data);
                }
            }
        }
    }
}
