#include "audio_player.h"
#include "nrf_drv_i2s.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_log.h"
#include <string.h>
#include "audio1.h"
#include "audio2.h"

/**
 * @brief Audio player instance structure
 */
typedef struct
{
    audio_state_t state;                                /**< Current state */
    uint8_t current_melody_id;                          /**< Currently playing melody ID */
    const int16_t *p_current_data;                      /**< Pointer to current melody data */
    uint32_t current_size;                              /**< Size of current melody */
    uint32_t current_position;                          /**< Current playback position */
    audio_playback_complete_handler_t complete_handler; /**< Completion callback */
} audio_player_t;

static audio_player_t m_audio_player = {0};

#define I2S_BUFFER_SIZE 512
static uint32_t m_i2s_tx_buffer[I2S_BUFFER_SIZE] __attribute__((aligned(4)));

static struct
{
    const uint16_t *melody_data;
    uint32_t melody_data_size;
} melodies[] = {
    [AUDIO_CMD_PLAY_MELODY_1].melody_data = melody_1_data,
    [AUDIO_CMD_PLAY_MELODY_1].melody_data_size = MELODY_1_DATA_SIZE,
    [AUDIO_CMD_PLAY_MELODY_2].melody_data = melody_2_data,
    [AUDIO_CMD_PLAY_MELODY_2].melody_data_size = MELODY_2_DATA_SIZE,
};

/**
 * @brief Fill I2S buffer with audio samples
 *
 * Converts 16-bit mono samples to 32-bit stereo format required by I2S
 */
static void fill_i2s_buffer(void)
{
    uint32_t remaining = m_audio_player.current_size - m_audio_player.current_position;
    uint32_t to_copy = (remaining < I2S_BUFFER_SIZE) ? remaining : I2S_BUFFER_SIZE;

    for (uint32_t i = 0; i < to_copy; i++)
    {
        int16_t sample = m_audio_player.p_current_data[m_audio_player.current_position++];

        m_i2s_tx_buffer[i] = ((uint32_t)sample << 16) | (uint16_t)sample;
    }

    for (uint32_t i = to_copy; i < I2S_BUFFER_SIZE; i++)
    {
        m_i2s_tx_buffer[i] = 0;
    }
}

/**
 * @brief I2S interrupt handler - called by DMA when buffer is consumed
 */
static void i2s_data_handler(nrf_drv_i2s_buffers_t const *p_released, uint32_t status)
{
    // DMA needs next buffer?
    if (status != NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
    {
        return;
    }

    // Check if we're done playing
    if (m_audio_player.current_position >= m_audio_player.current_size)
    {
        nrf_drv_i2s_stop();
        m_audio_player.state = AUDIO_STATE_IDLE;

        if (m_audio_player.complete_handler != NULL)
        {
            m_audio_player.complete_handler(m_audio_player.current_melody_id);
        }
        return;
    }

    // Prepare next chunk of audio
    fill_i2s_buffer();

    // Give buffer to DMA
    nrf_drv_i2s_buffers_t buffers = {
        .p_tx_buffer = m_i2s_tx_buffer,
        .p_rx_buffer = NULL,
    };

    ret_code_t err = nrf_drv_i2s_next_buffers_set(&buffers);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("I2S buffer set failed: %d", err);
    }
}

uint32_t audio_player_init(const audio_player_init_t *p_config)
{
    ret_code_t err_code;

    if (p_config == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (m_audio_player.state != AUDIO_STATE_UNINITIALIZED)
    {
        NRF_LOG_WARNING("Audio player already initialized");
        return NRF_SUCCESS;
    }

    m_audio_player.complete_handler = p_config->playback_complete_handler;

    // Configure I2S
    nrf_drv_i2s_config_t i2s_config = NRF_DRV_I2S_DEFAULT_CONFIG;
    i2s_config.sck_pin = I2S_CONFIG_SCK_PIN;
    i2s_config.lrck_pin = I2S_CONFIG_LRCK_PIN;
    i2s_config.sdout_pin = I2S_CONFIG_SDOUT_PIN;
    i2s_config.mck_pin = I2S_CONFIG_MCK_PIN;
    i2s_config.sample_width = NRF_I2S_SWIDTH_16BIT;
    i2s_config.channels = NRF_I2S_CHANNELS_STEREO;
    i2s_config.mck_setup = NRF_I2S_MCK_32MDIV8; // 4 MHz MCK
    i2s_config.ratio = NRF_I2S_RATIO_512X;      // 8 kHz sample rate

    err_code = nrf_drv_i2s_init(&i2s_config, i2s_data_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("I2S initialization failed: %d", err_code);
        return err_code;
    }
    m_audio_player.state = AUDIO_STATE_IDLE;

    NRF_LOG_INFO("Audio player initialized");

    return NRF_SUCCESS;
}

uint32_t audio_player_play(uint8_t melody_id)
{
    if (m_audio_player.state == AUDIO_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_MODULE_NOT_INITIALIZED;
    }

    if (melody_id < AUDIO_CMD_PLAY_MELODY_1 || AUDIO_CMD_PLAY_MELODY_2 < melody_id)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_audio_player.p_current_data = melodies[melody_id].melody_data;
    m_audio_player.current_size = melodies[melody_id].melody_data_size;
    m_audio_player.current_melody_id = melody_id;
    m_audio_player.current_position = 0;

    if (m_audio_player.state == AUDIO_STATE_IDLE)
    {
        // Prepare first buffer
        fill_i2s_buffer();

        // Start I2S transmission
        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_i2s_tx_buffer,
            .p_rx_buffer = NULL,
        };

        ret_code_t err_code = nrf_drv_i2s_start(&initial_buffers, I2S_BUFFER_SIZE, 0);
        m_audio_player.state = AUDIO_STATE_PLAYING;
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Failed to start I2S: %d", err_code);
            m_audio_player.state = AUDIO_STATE_IDLE;
            return err_code;
        }
    }

    NRF_LOG_INFO("Playing melody %d", melody_id);
    return NRF_SUCCESS;
}

uint32_t audio_player_stop(void)
{
    if (m_audio_player.state == AUDIO_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_MODULE_NOT_INITIALIZED;
    }

    if (m_audio_player.state != AUDIO_STATE_PLAYING)
    {
        return NRF_SUCCESS; // Already stopped
    }

    m_audio_player.state = AUDIO_STATE_STOPPING;
    nrf_drv_i2s_stop();
    m_audio_player.state = AUDIO_STATE_IDLE;
    m_audio_player.current_position = 0;

    NRF_LOG_INFO("Playback stopped");
    return NRF_SUCCESS;
}

audio_state_t audio_player_get_state(void)
{
    return m_audio_player.state;
}