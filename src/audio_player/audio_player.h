/**
 * @file audio_player.h
 * @brief I2S audio player for playing melodies stored in flash
 */

#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include <stdint.h>

/**
 * @brief Audio playback commands
 */
typedef enum
{
    AUDIO_CMD_PLAY_MELODY_1 = 0x01, /**< Play first melody */
    AUDIO_CMD_PLAY_MELODY_2 = 0x02, /**< Play second melody */
    AUDIO_CMD_STOP = 0x03,          /**< Stop playback */
} audio_command_t;

/**
 * @brief Audio player state
 */
typedef enum
{
    AUDIO_STATE_IDLE,     /**< Not playing */
    AUDIO_STATE_PLAYING,  /**< Currently playing */
    AUDIO_STATE_STOPPING, /**< Stopping playback */
} audio_state_t;

/**
 * @brief Callback function type for playback completion
 *
 * @param[in] melody_id ID of the melody that finished playing (1 or 2)
 */
typedef void (*audio_playback_complete_handler_t)(uint8_t melody_id);

/**
 * @brief Audio player configuration structure
 */
typedef struct
{
    audio_playback_complete_handler_t playback_complete_handler; /**< Callback when playback completes */
} audio_player_init_t;

/**
 * @brief Initialize the audio player
 *
 * @param[in] p_config Pointer to configuration structure
 *
 * @retval NRF_SUCCESS          Audio player initialized successfully
 * @retval NRF_ERROR_NULL       NULL pointer supplied
 * @retval NRF_ERROR_INTERNAL   Initialization failed
 */
uint32_t audio_player_init(const audio_player_init_t *p_config);

/**
 * @brief Play a melody
 *
 * @param[in] melody_id Melody ID (1 or 2)
 *
 * @retval NRF_SUCCESS              Playback started successfully
 * @retval NRF_ERROR_INVALID_PARAM  Invalid melody ID
 * @retval NRF_ERROR_BUSY           Already playing
 */
uint32_t audio_player_play(uint8_t melody_id);

/**
 * @brief Stop current playback
 *
 * @retval NRF_SUCCESS Playback stopped successfully
 */
uint32_t audio_player_stop(void);

/**
 * @brief Get current audio player state
 *
 * @return Current state
 */
audio_state_t audio_player_get_state(void);

#endif /* AUDIO_PLAYER_H */