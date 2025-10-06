/**
 * @file accelerometer.h
 * @brief BMA280 accelerometer driver with interrupt support
 */

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>
#include <stdbool.h>

#define BMA280_I2C_ADRESS 0x18

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

/**
 * @brief Accelerometer data structure (X, Y, Z axes)
 */
typedef struct
{
    int16_t x; /**< X-axis value in mg (milligravity) */
    int16_t y; /**< Y-axis value in mg */
    int16_t z; /**< Z-axis value in mg */
} accelerometer_data_t;

/**
 * @brief Interrupt event callback type
 *
 * Called when motion is detected (from external interrupt pin)
 *
 * @param[in] p_data Pointer to latest accelerometer data
 */
typedef void (*accelerometer_motion_handler_t)(const accelerometer_data_t *p_data);

/**
 * @brief Accelerometer configuration
 */
typedef struct
{
    uint8_t i2c_address;                                 /**< I2C device address */
    accelerometer_motion_handler_t motion_handler; /**< Motion interrupt callback */
} accelerometer_init_t;

/**
 * @brief Initialize BMA280 accelerometer
 *
 * @param[in] p_config Pointer to configuration
 *
 * @retval NRF_SUCCESS          Initialized successfully
 * @retval NRF_ERROR_NULL       NULL pointer
 * @retval NRF_ERROR_INTERNAL   I2C or sensor initialization failed
 */
uint32_t accelerometer_init(const accelerometer_init_t *p_config);

/**
 * @brief Read current accelerometer data
 *
 * @param[out] p_data Pointer to data structure to fill
 *
 * @retval NRF_SUCCESS      Data read successfully
 * @retval NRF_ERROR_NULL   NULL pointer
 * @retval NRF_ERROR_BUSY   I2C bus busy
 */
uint32_t accelerometer_read(accelerometer_data_t *p_data);

/**
 * @brief Get last cached accelerometer data
 *
 * Returns data from last interrupt/read without new I2C transaction
 *
 * @param[out] p_data Pointer to data structure to fill
 *
 * @retval NRF_SUCCESS      Data retrieved
 * @retval NRF_ERROR_NULL   NULL pointer
 */
uint32_t accelerometer_get_cached_data(accelerometer_data_t *p_data);

/**
 * @brief Convert accelerometer data to 6-byte array for BLE
 *
 * Format: [X_low, X_high, Y_low, Y_high, Z_low, Z_high]
 *
 * @param[in]  p_data   Pointer to accelerometer data
 * @param[out] p_buffer Pointer to 6-byte output buffer
 */
void accelerometer_data_to_bytes(const accelerometer_data_t *p_data, uint8_t *p_buffer);

/**
 * @brief Enable motion detection interrupt
 *
 * @retval NRF_SUCCESS Configuration updated successfully
 */
uint32_t accelerometer_enable_interrupt(void);

/**
 * @brief Disable motion detection interrupt
 *
 * @retval NRF_SUCCESS Configuration updated successfully
 */
uint32_t accelerometer_disable_interrupt(void);

/**
 * @brief Process accelerometer events
 * 
 * Call periodically from main loop to check for motion events.
 * Reads INT_STATUS register and triggers callback if motion detected.
 */
void accelerometer_process_event(void);


#endif /* ACCELEROMETER_H */