#ifndef BLE_AUDIO_ACC_SERVICE_H
#define BLE_AUDIO_ACC_SERVICE_H

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/**
 * @file ble_audio_acc_service.h
 * @brief BLE service for audio playback control and accelerometer data reading
 */

/**
 * @brief Macro for defining an audio/accelerometer service instance
 */
#define BLE_AUDIO_ACC_DEF(_name)                          \
    static ble_audio_acc_service_t _name;                 \
    NRF_SDH_BLE_OBSERVER(_name##_obs,                     \
                         BLE_AUDIO_ACC_BLE_OBSERVER_PRIO, \
                         ble_audio_acc_on_ble_evt, &_name)

// Custom UUID base: fc57430c-6271-4c62-abed-540a6a9aa9ef
#define AAS_UUID_BASE {0xEF, 0xA9, 0x9A, 0x6A, 0x0A, 0x54, 0xED, 0xAB, \
                       0x62, 0x4C, 0x71, 0x62, 0x0C, 0x43, 0x57, 0xFC}

#define AAS_SERVICE_UUID 0x1400            /**< Service UUID */
#define AAS_UUID_AUDIO_CHAR 0x1401         /**< Audio control characteristic UUID (write-only) */
#define AAS_UUID_ACCELEROMETER_CHAR 0x1402 /**< Accelerometer characteristic UUID (read-only) */

// Forward declaration
typedef struct ble_audio_acc_service_s ble_audio_acc_service_t;

/**
 * @brief Audio control write handler type
 *
 * @param[in] conn_handle Connection handle
 * @param[in] p_service   Pointer to service instance
 * @param[in] command     Audio command byte (0x01, 0x02, or 0x03)
 */
typedef void (*ble_audio_write_handler_t)(uint16_t conn_handle,
                                          ble_audio_acc_service_t *p_service,
                                          uint8_t command);

/**
 * @brief Service initialization structure
 */
typedef struct
{
    ble_audio_write_handler_t audio_write_handler; /**< Handler called when audio characteristic is written */
} ble_audio_acc_init_t;

/**
 * @brief Audio/Accelerometer service structure
 */
struct ble_audio_acc_service_s
{
    uint16_t service_handle;                       /**< Handle of the service */
    ble_gatts_char_handles_t audio_char_handles;   /**< Handles for audio characteristic */
    ble_gatts_char_handles_t acc_char_handles;     /**< Handles for accelerometer characteristic */
    uint8_t uuid_type;                             /**< UUID type for the service */
    ble_audio_write_handler_t audio_write_handler; /**< Audio write event handler */
};

/**
 * @brief Initialize the Audio/Accelerometer service
 *
 * @param[out] p_service Pointer to service structure
 * @param[in]  p_init    Pointer to initialization structure
 *
 * @retval NRF_SUCCESS          Service initialized successfully
 * @retval NRF_ERROR_NULL       NULL pointer supplied
 * @retval NRF_ERROR_INTERNAL   Service initialization failed
 */
uint32_t ble_audio_acc_init(ble_audio_acc_service_t *p_service,
                            const ble_audio_acc_init_t *p_init);

/**
 * @brief Handle BLE stack events
 *
 * @param[in] p_ble_evt Event received from the BLE stack
 * @param[in] p_context Pointer to service structure
 */
void ble_audio_acc_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**
 * @brief Update accelerometer characteristic value
 *
 * @param[in] conn_handle Connection handle
 * @param[in] p_service   Pointer to service structure
 * @param[in] p_acc_data  Pointer to 6-byte accelerometer data (x, y, z as int16_t)
 *
 * @retval NRF_SUCCESS              Notification sent successfully
 * @retval NRF_ERROR_INVALID_STATE  Invalid connection state
 * @retval NRF_ERROR_NULL           NULL pointer supplied
 */
uint32_t ble_audio_acc_accelerometer_update(uint16_t conn_handle,
                                            ble_audio_acc_service_t *p_service,
                                            const uint8_t *p_acc_data);

#endif /* BLE_AUDIO_ACC_SERVICE_H */