#include "ble_audio_acc_service.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_log.h"

/**
 * @brief Handle write events for audio characteristic
 *
 * @param[in] p_service Pointer to service structure
 * @param[in] p_ble_evt Pointer to BLE event with write event data
 */
static void on_audio_char_write(ble_audio_acc_service_t *p_service, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_service->audio_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_service->audio_write_handler != NULL))
    {
        p_service->audio_write_handler(p_ble_evt->evt.gatts_evt.conn_handle,
                                       p_service,
                                       p_evt_write->data[0]);
    }
}

/**
 * @brief Add audio control characteristic to the service
 *
 * @param[in] p_service Pointer to service structure
 *
 * @retval NRF_SUCCESS Characteristic added successfully
 */
static uint32_t audio_char_add(ble_audio_acc_service_t *p_service)
{
    ble_gatts_char_md_t char_md = {0};
    ble_gatts_attr_t attr_char_value = {0};
    ble_uuid_t ble_uuid = {0};
    ble_gatts_attr_md_t attr_md = {0};

    // Characteristic properties: write only (no response)
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;

    // Configure UUID
    ble_uuid.type = p_service->uuid_type;
    ble_uuid.uuid = AAS_UUID_AUDIO_CHAR;

    // Configure attribute metadata
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Configure attribute value
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(uint8_t);
    attr_char_value.max_len = sizeof(uint8_t);

    return sd_ble_gatts_characteristic_add(p_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_service->audio_char_handles);
}

/**
 * @brief Add accelerometer characteristic to the service
 *
 * @param[in] p_service Pointer to service structure
 *
 * @retval NRF_SUCCESS Characteristic added successfully
 */
static uint32_t accelerometer_char_add(ble_audio_acc_service_t *p_service)
{
    ble_gatts_char_md_t char_md = {0};
    ble_gatts_attr_t attr_char_value = {0};
    ble_uuid_t ble_uuid = {0};
    ble_gatts_attr_md_t attr_md = {0};

    // Characteristic properties: read only capability
    char_md.char_props.read = 1;

    // Configure UUID
    ble_uuid.type = p_service->uuid_type;
    ble_uuid.uuid = AAS_UUID_ACCELEROMETER_CHAR;

    // Configure attribute metadata
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Configure attribute value (6 bytes for x, y, z as int16_t)
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 6;
    attr_char_value.max_len = 6;

    return sd_ble_gatts_characteristic_add(p_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_service->acc_char_handles);
}

uint32_t ble_audio_acc_init(ble_audio_acc_service_t *p_service,
                            const ble_audio_acc_init_t *p_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid = {0};
    ble_uuid128_t base_uuid = {AAS_UUID_BASE};

    // Validate input parameters
    if ((p_service == NULL) || (p_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure
    p_service->audio_write_handler = p_init->audio_write_handler;

    // Add custom base UUID
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_service->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to add vendor specific UUID: %d", err_code);
        return err_code;
    }

    // Configure service UUID
    ble_uuid.type = p_service->uuid_type;
    ble_uuid.uuid = AAS_SERVICE_UUID;

    // Add service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_service->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to add service: %d", err_code);
        return err_code;
    }

    // Add audio characteristic
    err_code = audio_char_add(p_service);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to add audio characteristic: %d", err_code);
        return err_code;
    }

    // Add accelerometer characteristic
    err_code = accelerometer_char_add(p_service);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to add accelerometer characteristic: %d", err_code);
        return err_code;
    }

    NRF_LOG_INFO("Audio/Accelerometer service initialized successfully");
    return NRF_SUCCESS;
}

void ble_audio_acc_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    ble_audio_acc_service_t *p_service = (ble_audio_acc_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GATTS_EVT_WRITE:
        on_audio_char_write(p_service, p_ble_evt);
        break;
    default:
        break;
    }
}

uint32_t ble_audio_acc_accelerometer_update(uint16_t conn_handle,
                                            ble_audio_acc_service_t *p_service,
                                            const uint8_t *p_acc_data)
{
    uint32_t err_code;

    if ((p_service == NULL) || (p_acc_data == NULL))
    {
        return NRF_ERROR_NULL;
    }

    if (conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gatts_value_t gatts_value = {0};
    gatts_value.len = 6;
    gatts_value.p_value = (uint8_t *)p_acc_data;

    err_code = sd_ble_gatts_value_set(conn_handle,
                                      p_service->acc_char_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to update GATT value: %d", err_code);
        return err_code;
    }

    return err_code;
}