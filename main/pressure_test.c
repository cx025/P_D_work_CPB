#include <errno.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "ble_control.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "pressure_sensor.h"
#include "pcf8563.h"
#include "drv8833.h"
#include "record_log.h"
#include "speaker.h"

#define STARTUP_VIBRATION_MS 500
#define STARTUP_VIBRATION_AMPLITUDE DRV8833_MAX_LEVEL
#define STARTUP_VIBRATION_HZ 80
/* The level-two pulse is delayed to account for MP3 decode startup latency. */
#define BLAST_VIBRATION_DELAY_MS 760
#define FIRST_LEVEL_VIBRATION_MS 950
#define FIRST_LEVEL_VIBRATION_AMPLITUDE DRV8833_MAX_LEVEL
#define SECOND_LEVEL_VIBRATION_AMPLITUDE DRV8833_MAX_LEVEL
#define FIRST_LEVEL_VIBRATION_HZ 80
#define AWKWARD_VIBRATION_DEFAULT_HZ 160
#define AWKWARD_SHORT_VIBRATION_HZ 55
#define AWKWARD_LONG_VIBRATION_HZ 60
#define AWKWARD_TIMELINE_AMPLITUDE 70
#define AWKWARD_SHORT_VIBRATION_MS 115
#define AWKWARD_EXTENDED_PULSE_INDEX 11U
#define AWKWARD_EXTENDED_PULSE_MS 146U
#define AWKWARD_FINAL_VIBRATION_MS 136

#define AUDIO_CONFIG_NAMESPACE "audio"
#define AUDIO_CONFIG_KEY "settings"
#define AUDIO_ENABLED_KEY "enabled"
#define DAILY_STATS_KEY "daily"
#define MOTOR_FREQUENCY_KEY "motor_hz"
#define BLAST_FREQUENCY_KEY "blast_hz"
#define MOTOR_AMPLITUDE_KEY "motor_amp"
#define BLAST_AMPLITUDE_KEY "blast_amp"
#define MIN_REMINDER_MS 60000
#define MAX_REMINDER_MS 86400000
#define MIN_MOTOR_FREQUENCY_HZ 40
#define MAX_MOTOR_FREQUENCY_HZ DRV8833_MAX_VIBRATION_HZ
#define MIN_MOTOR_AMPLITUDE 1U
#define MANUAL_VIBRATION_MIN_MS 100U
#define MANUAL_VIBRATION_MIN_LEVEL 1U
#define SEAT_CONFIRM_MS 30000
#define LEAVE_CONFIRM_MS 60000
#define LEVEL_THREE_REPEAT_MS 3600000
#define SESSION_NAMESPACE "session"
#define SESSION_CHECKPOINT_KEY "checkpoint"
#define SYNC_BATCH_SIZE 8
#define STATUS_HEARTBEAT_MS 30000

typedef struct {
    uint8_t volume_percent;
    uint32_t delay_ms[3];
} audio_settings_t;

typedef struct {
    uint32_t day_key;
    uint32_t sitting_seconds;
    uint32_t sessions;
    uint32_t reminders;
    uint32_t last_session_seconds;
    uint32_t last_session_reminders;
    uint8_t last_session_responded;
    uint8_t version;
} daily_stats_t;

/* Persisted in NVS so a session interrupted by power loss can be recorded on
 * the next boot together with how long the user had been sitting. */
typedef struct {
    uint8_t active;
    uint32_t start_epoch;
    uint32_t start_uptime;
    uint32_t seated_seconds;
    uint32_t session_seq;
} session_checkpoint_t;

typedef enum {
    CONTROL_SETTINGS,
    CONTROL_ENABLED,
    CONTROL_REMINDER,
    CONTROL_STOP_ALL,
    CONTROL_CLOCK,
    CONTROL_MOTOR_FREQUENCY,
    CONTROL_VIBRATE,
    CONTROL_SYNC,
    CONTROL_ACK,
    CONTROL_CLEAR_LOG,
} control_type_t;

typedef struct {
    control_type_t type;
    audio_settings_t settings;
    bool enabled;
    uint8_t reminder_stage;
    uint32_t local_epoch_seconds;
    uint16_t blast_frequency_hz;
    uint16_t motor_frequency_hz;
    uint8_t blast_amplitude;
    uint8_t motor_amplitude;
    uint32_t vibrate_duration_ms;
    uint8_t vibrate_level;
    uint8_t vibrate_frequency_hz;
    uint32_t sequence;
} control_command_t;

static const char *TAG = "pressure_audio";
static const audio_settings_t DEFAULT_SETTINGS = {
    .volume_percent = 70,
    .delay_ms = {1800000, 3600000, 5400000},
};
static QueueHandle_t control_queue;
static audio_settings_t audio_settings;
static daily_stats_t daily_stats;
static uint16_t blast_frequency_hz = FIRST_LEVEL_VIBRATION_HZ;
static uint16_t motor_frequency_hz = AWKWARD_VIBRATION_DEFAULT_HZ;
static uint8_t blast_amplitude = FIRST_LEVEL_VIBRATION_AMPLITUDE;
static uint8_t motor_amplitude = SECOND_LEVEL_VIBRATION_AMPLITUDE;
static bool settings_saved;
static bool motor_settings_saved = true;
static bool reminders_enabled = true;
static bool reminders_saved = true;
static bool clock_synced;
static bool clock_read_error_reported;
static bool motor_ready;
static bool session_active;
static bool awkward_vibration_timeline_active;
static unsigned awkward_timeline_frequency_hz;
static bool vibration_pending;
static int64_t vibration_fire_at_us;
static uint32_t pending_vibration_duration_ms;
static unsigned pending_vibration_level;
static unsigned pending_vibration_hz;
static uint32_t session_seconds;
static uint32_t session_reminder_stage;
static uint32_t session_reminders;
static uint32_t daily_counted_session_seconds;
static uint64_t session_seated_milliseconds;
static uint64_t last_repeat_milliseconds;
static TickType_t seat_pending_since;
static TickType_t leave_pending_since;
static TickType_t previous_tick;
static uint32_t last_persisted_sitting_seconds;
static bool sync_active;
static uint32_t sync_next_seq;
static uint32_t current_session_seq;
static uint32_t session_start_epoch;
static uint32_t session_start_uptime;

/* Short-pulse start offsets for the awkward clip, in milliseconds. */
static const uint16_t awkward_short_vibration_starts_ms[] = {
    2771,  3233,  3694,  4156,  4618,  5080,  5542,  6003,
    6465,  6927,  7389,  7850,  8312,  8774,  9236,  9698,
    10159, 10621, 11083, 11545, 12007, 12468, 12930, 13392,
    13854, 14316, 14777, 15239, 15701, 16163, 16625, 17086,
    17548, 18010, 18472, 18934, 19395, 19857, 20319, 20781,
    21243, 21704, 22166, 22628, 23090, 23551, 24013, 24475,
    24937, 25399, 25860, 26322, 26784, 27246, 27708, 28169,
    28631, 29093, 29555, 30017, 30478, 30940, 31402, 31864,
};

static esp_err_t save_settings(const audio_settings_t *settings);

static esp_err_t save_motor_settings(uint16_t blast_hz, uint16_t awkward_hz,
                                     uint8_t blast_amp, uint8_t awkward_amp)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open motor settings for writing: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = nvs_set_u16(handle, BLAST_FREQUENCY_KEY, blast_hz);
    if (err == ESP_OK) {
        err = nvs_set_u16(handle, MOTOR_FREQUENCY_KEY, awkward_hz);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, BLAST_AMPLITUDE_KEY, blast_amp);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, MOTOR_AMPLITUDE_KEY, awkward_amp);
    }
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save motor settings: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static bool load_motor_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) return true;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open motor settings: %s", esp_err_to_name(err));
        return false;
    }
    uint16_t saved_awkward_hz = motor_frequency_hz;
    esp_err_t awkward_err =
        nvs_get_u16(handle, MOTOR_FREQUENCY_KEY, &saved_awkward_hz);
    uint16_t saved_blast_hz = blast_frequency_hz;
    esp_err_t blast_err =
        nvs_get_u16(handle, BLAST_FREQUENCY_KEY, &saved_blast_hz);
    uint8_t saved_awkward_amp = motor_amplitude;
    esp_err_t awkward_amp_err =
        nvs_get_u8(handle, MOTOR_AMPLITUDE_KEY, &saved_awkward_amp);
    uint8_t saved_blast_amp = blast_amplitude;
    esp_err_t blast_amp_err =
        nvs_get_u8(handle, BLAST_AMPLITUDE_KEY, &saved_blast_amp);
    nvs_close(handle);
    if ((awkward_err != ESP_OK && awkward_err != ESP_ERR_NVS_NOT_FOUND) ||
        (blast_err != ESP_OK && blast_err != ESP_ERR_NVS_NOT_FOUND) ||
        (awkward_amp_err != ESP_OK &&
         awkward_amp_err != ESP_ERR_NVS_NOT_FOUND) ||
        (blast_amp_err != ESP_OK && blast_amp_err != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE(TAG, "Could not read motor settings: %s",
                 esp_err_to_name(awkward_err != ESP_OK &&
                                         awkward_err != ESP_ERR_NVS_NOT_FOUND
                                     ? awkward_err
                                 : blast_err != ESP_OK &&
                                           blast_err != ESP_ERR_NVS_NOT_FOUND
                                     ? blast_err
                                 : awkward_amp_err != ESP_OK &&
                                           awkward_amp_err !=
                                               ESP_ERR_NVS_NOT_FOUND
                                     ? awkward_amp_err
                                     : blast_amp_err));
        return false;
    }
    bool repair_saved_settings =
        awkward_err == ESP_ERR_NVS_NOT_FOUND ||
        blast_err == ESP_ERR_NVS_NOT_FOUND ||
        awkward_amp_err == ESP_ERR_NVS_NOT_FOUND ||
        blast_amp_err == ESP_ERR_NVS_NOT_FOUND;
    if (awkward_err == ESP_OK && saved_awkward_hz == 150) {
        saved_awkward_hz = AWKWARD_VIBRATION_DEFAULT_HZ;
        repair_saved_settings = true;
    }
    if (saved_awkward_hz < MIN_MOTOR_FREQUENCY_HZ ||
        saved_awkward_hz > MAX_MOTOR_FREQUENCY_HZ) {
        ESP_LOGW(TAG, "Saved awkward vibration frequency is invalid; restoring default.");
        saved_awkward_hz = AWKWARD_VIBRATION_DEFAULT_HZ;
        repair_saved_settings = true;
    }
    if (saved_blast_hz < MIN_MOTOR_FREQUENCY_HZ ||
        saved_blast_hz > MAX_MOTOR_FREQUENCY_HZ) {
        ESP_LOGW(TAG, "Saved blast vibration frequency is invalid; restoring default.");
        saved_blast_hz = FIRST_LEVEL_VIBRATION_HZ;
        repair_saved_settings = true;
    }
    if (saved_awkward_amp < MIN_MOTOR_AMPLITUDE ||
        saved_awkward_amp > DRV8833_MAX_LEVEL) {
        ESP_LOGW(TAG, "Saved awkward vibration amplitude is invalid; restoring default.");
        saved_awkward_amp = SECOND_LEVEL_VIBRATION_AMPLITUDE;
        repair_saved_settings = true;
    }
    if (saved_blast_amp < MIN_MOTOR_AMPLITUDE ||
        saved_blast_amp > DRV8833_MAX_LEVEL) {
        ESP_LOGW(TAG, "Saved blast vibration amplitude is invalid; restoring default.");
        saved_blast_amp = FIRST_LEVEL_VIBRATION_AMPLITUDE;
        repair_saved_settings = true;
    }
    if (repair_saved_settings) {
        if (save_motor_settings(saved_blast_hz, saved_awkward_hz,
                                saved_blast_amp, saved_awkward_amp) != ESP_OK) {
            return false;
        }
    }
    blast_frequency_hz = saved_blast_hz;
    motor_frequency_hz = saved_awkward_hz;
    blast_amplitude = saved_blast_amp;
    motor_amplitude = saved_awkward_amp;
    return true;
}

static bool settings_are_valid(const audio_settings_t *settings)
{
    return settings->volume_percent <= 100 &&
           settings->delay_ms[0] >= MIN_REMINDER_MS &&
           settings->delay_ms[0] <= settings->delay_ms[1] &&
           settings->delay_ms[1] <= settings->delay_ms[2] &&
           settings->delay_ms[2] <= MAX_REMINDER_MS;
}

static bool load_settings(audio_settings_t *settings)
{
    *settings = DEFAULT_SETTINGS;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) return true;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open audio settings: %s", esp_err_to_name(err));
        return false;
    }

    size_t size = sizeof(*settings);
    err = nvs_get_blob(handle, AUDIO_CONFIG_KEY, settings, &size);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *settings = DEFAULT_SETTINGS;
        return true;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read audio settings: %s", esp_err_to_name(err));
        *settings = DEFAULT_SETTINGS;
        return false;
    }
    if (size != sizeof(*settings) || !settings_are_valid(settings)) {
        ESP_LOGW(TAG, "Saved timings are invalid for long-sitting reminders; using safe defaults.");
        *settings = DEFAULT_SETTINGS;
        return save_settings(settings) == ESP_OK;
    }
    return true;
}

static esp_err_t save_settings(const audio_settings_t *settings)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open audio settings for writing: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(handle, AUDIO_CONFIG_KEY, settings, sizeof(*settings));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save audio settings: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t save_enabled(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open reminder switch for writing: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = nvs_set_u8(handle, AUDIO_ENABLED_KEY, enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save reminder switch: %s", esp_err_to_name(err));
    }
    return err;
}

static bool load_enabled(bool *saved)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *saved = true;
        return true;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read reminder switch: %s", esp_err_to_name(err));
        *saved = false;
        return true;
    }
    uint8_t enabled = 1;
    err = nvs_get_u8(handle, AUDIO_ENABLED_KEY, &enabled);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *saved = true;
        return true;
    }
    if (err != ESP_OK || enabled > 1) {
        ESP_LOGE(TAG, "Saved reminder switch is invalid; reminders remain enabled.");
        *saved = false;
        return true;
    }
    *saved = true;
    return enabled != 0;
}

static esp_err_t save_daily_stats(void)
{
    daily_stats.version = 1;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open daily statistics for writing: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(handle, DAILY_STATS_KEY, &daily_stats, sizeof(daily_stats));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save daily statistics: %s", esp_err_to_name(err));
    }
    return err;
}

static void load_daily_stats(void)
{
    memset(&daily_stats, 0, sizeof(daily_stats));
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUDIO_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) return;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open daily statistics: %s", esp_err_to_name(err));
        return;
    }
    size_t size = sizeof(daily_stats);
    err = nvs_get_blob(handle, DAILY_STATS_KEY, &daily_stats, &size);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        memset(&daily_stats, 0, sizeof(daily_stats));
    } else if (err != ESP_OK || size != sizeof(daily_stats) ||
               daily_stats.version != 1) {
        ESP_LOGE(TAG, "Saved daily statistics are invalid; starting fresh.");
        memset(&daily_stats, 0, sizeof(daily_stats));
    }
    last_persisted_sitting_seconds = daily_stats.sitting_seconds;
}

static void publish_status(bool pressure)
{
    ble_control_update_audio_status(
        pressure, audio_settings.volume_percent, audio_settings.delay_ms[0],
        audio_settings.delay_ms[1], audio_settings.delay_ms[2], settings_saved,
        reminders_enabled, reminders_saved, session_active,
        session_active && !pressure, session_seconds,
        daily_stats.sitting_seconds, daily_stats.sessions, daily_stats.reminders,
        daily_stats.last_session_seconds, daily_stats.last_session_reminders,
        daily_stats.last_session_responded != 0, clock_synced,
        blast_frequency_hz, motor_frequency_hz, blast_amplitude,
        motor_amplitude, motor_settings_saved);
}

static bool parse_uint32(char **cursor, uint32_t *value, bool require_space)
{
    while (**cursor == ' ' || **cursor == '\t') ++*cursor;
    if (**cursor == '\0' || **cursor == '-') return false;

    char *end = NULL;
    const unsigned long parsed = strtoul(*cursor, &end, 10);
    if (end == *cursor || parsed > UINT32_MAX) return false;
    if (require_space && *end != ' ' && *end != '\t') return false;
    if (!require_space && *end != '\0' && *end != '\r' && *end != '\n') {
        return false;
    }
    *value = (uint32_t)parsed;
    *cursor = end;
    return true;
}

static esp_err_t queue_control(const control_command_t *command)
{
    return xQueueSend(control_queue, command, 0) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

esp_err_t process_terminal_line(char *line)
{
    char *cursor = line;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;

    if (strncmp(cursor, "AUDIO", 5) == 0 &&
        (cursor[5] == ' ' || cursor[5] == '\t')) {
        cursor += 5;
        uint32_t values[4];
        for (size_t i = 0; i < 4; ++i) {
            if (!parse_uint32(&cursor, &values[i], i < 3)) {
                ESP_LOGW(TAG, "Invalid audio settings command.");
                return ESP_ERR_INVALID_ARG;
            }
        }
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
               *cursor == '\n') ++cursor;
        const audio_settings_t settings = {
            .volume_percent = (uint8_t)values[0],
            .delay_ms = {values[1], values[2], values[3]},
        };
        if (*cursor != '\0' || values[0] > 100 ||
            !settings_are_valid(&settings)) {
            ESP_LOGW(TAG, "Volume must be 0-100 and reminder thresholds must be ascending, 1-1440 minutes.");
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {.type = CONTROL_SETTINGS, .settings = settings};
        return queue_control(&command);
    }

    if (strncmp(cursor, "POWER ", 6) == 0) {
        cursor += 6;
        control_command_t command = {.type = CONTROL_ENABLED};
        if (strcmp(cursor, "ON") == 0) command.enabled = true;
        else if (strcmp(cursor, "OFF") == 0) command.enabled = false;
        else return ESP_ERR_INVALID_ARG;
        return queue_control(&command);
    }

    if (strncmp(cursor, "STOP ALL", 8) == 0 &&
        (cursor[8] == '\0' || cursor[8] == '\r' || cursor[8] == '\n' ||
         cursor[8] == ' ' || cursor[8] == '\t')) {
        cursor += 8;
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
               *cursor == '\n') ++cursor;
        if (*cursor != '\0') {
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {.type = CONTROL_STOP_ALL};
        return queue_control(&command);
    }

    if (strncmp(cursor, "REMINDER ", 9) == 0) {
        cursor += 9;
        uint32_t stage;
        if (!parse_uint32(&cursor, &stage, false) || stage < 1 || stage > 3) {
            ESP_LOGW(TAG, "REMINDER stage must be 1, 2, or 3.");
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {
            .type = CONTROL_REMINDER,
            .reminder_stage = (uint8_t)stage,
        };
        return queue_control(&command);
    }

    if (strncmp(cursor, "MOTOR ", 6) == 0) {
        cursor += 6;
        uint32_t blast_amp, blast_hz, awkward_amp, awkward_hz;
        if (!parse_uint32(&cursor, &blast_amp, true) ||
            !parse_uint32(&cursor, &blast_hz, true) ||
            !parse_uint32(&cursor, &awkward_amp, true) ||
            !parse_uint32(&cursor, &awkward_hz, false) ||
            blast_amp < MIN_MOTOR_AMPLITUDE ||
            blast_amp > DRV8833_MAX_LEVEL ||
            awkward_amp < MIN_MOTOR_AMPLITUDE ||
            awkward_amp > DRV8833_MAX_LEVEL ||
            blast_hz < MIN_MOTOR_FREQUENCY_HZ ||
            blast_hz > MAX_MOTOR_FREQUENCY_HZ ||
            awkward_hz < MIN_MOTOR_FREQUENCY_HZ ||
            awkward_hz > MAX_MOTOR_FREQUENCY_HZ) {
            ESP_LOGW(TAG, "Motor amplitudes must be 1-90 and frequencies 40-200 Hz.");
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {
            .type = CONTROL_MOTOR_FREQUENCY,
            .blast_amplitude = (uint8_t)blast_amp,
            .blast_frequency_hz = (uint16_t)blast_hz,
            .motor_amplitude = (uint8_t)awkward_amp,
            .motor_frequency_hz = (uint16_t)awkward_hz,
        };
        return queue_control(&command);
    }

    if (strncmp(cursor, "CLOCK ", 6) == 0) {
        cursor += 6;
        uint32_t local_epoch_seconds;
        if (!parse_uint32(&cursor, &local_epoch_seconds, false) ||
            local_epoch_seconds < 946684800U) {
            ESP_LOGW(TAG, "Invalid local clock synchronization command.");
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {
            .type = CONTROL_CLOCK,
            .local_epoch_seconds = local_epoch_seconds,
        };
        return queue_control(&command);
    }

    if (strncmp(cursor, "VIBRATE ", 8) == 0) {
        cursor += 8;
        control_command_t command = {.type = CONTROL_VIBRATE};
        if (strncmp(cursor, "STOP", 4) == 0 &&
            (cursor[4] == '\0' || cursor[4] == '\r' || cursor[4] == '\n')) {
            command.vibrate_duration_ms = 0;
            return queue_control(&command);
        }
        uint32_t amplitude, frequency, duration_ms;
        if (!parse_uint32(&cursor, &amplitude, true) ||
            !parse_uint32(&cursor, &frequency, true) ||
            !parse_uint32(&cursor, &duration_ms, false) ||
            amplitude < MANUAL_VIBRATION_MIN_LEVEL ||
            amplitude > DRV8833_MAX_LEVEL ||
            frequency < MIN_MOTOR_FREQUENCY_HZ ||
            frequency > MAX_MOTOR_FREQUENCY_HZ ||
            duration_ms < MANUAL_VIBRATION_MIN_MS ||
            duration_ms > DRV8833_MAX_TEST_MS) {
            ESP_LOGW(TAG,
                     "VIBRATE needs an amplitude of 1-%u, a frequency of "
                     "40-200 Hz, and a duration of %u-%u ms.",
                     (unsigned)DRV8833_MAX_LEVEL, MANUAL_VIBRATION_MIN_MS,
                     (unsigned)DRV8833_MAX_TEST_MS);
            return ESP_ERR_INVALID_ARG;
        }
        command.vibrate_level = (uint8_t)amplitude;
        command.vibrate_frequency_hz = (uint8_t)frequency;
        command.vibrate_duration_ms = duration_ms;
        return queue_control(&command);
    }

    if (strncmp(cursor, "SYNC ", 5) == 0) {
        cursor += 5;
        uint32_t after_seq;
        if (!parse_uint32(&cursor, &after_seq, false)) {
            ESP_LOGW(TAG, "Invalid history sync command.");
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {.type = CONTROL_SYNC,
                                     .sequence = after_seq};
        return queue_control(&command);
    }

    if (strncmp(cursor, "ACK ", 4) == 0) {
        cursor += 4;
        uint32_t seq;
        if (!parse_uint32(&cursor, &seq, false)) {
            ESP_LOGW(TAG, "Invalid history acknowledgement command.");
            return ESP_ERR_INVALID_ARG;
        }
        control_command_t command = {.type = CONTROL_ACK, .sequence = seq};
        return queue_control(&command);
    }

    if (strncmp(cursor, "CLEAR", 5) == 0 &&
        (cursor[5] == '\0' || cursor[5] == '\r' || cursor[5] == '\n' ||
         cursor[5] == ' ' || cursor[5] == '\t')) {
        control_command_t command = {.type = CONTROL_CLEAR_LOG};
        return queue_control(&command);
    }

    ESP_LOGW(TAG, "Unsupported BLE command.");
    return ESP_ERR_INVALID_ARG;
}

static bool play_clip(speaker_mp3_clip_t clip, const char *description)
{
    if (!speaker_is_ready()) {
        ESP_LOGE(TAG, "Cannot play %s: speaker is not ready.", description);
        return false;
    }
    if (speaker_is_song_playing()) {
        ESP_LOGW(TAG, "Skipped %s because another clip is still playing.",
                 description);
        return false;
    }
    const esp_err_t err = speaker_play_clip(clip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not play %s: %s", description, esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool start_vibration(uint32_t duration_ms, unsigned level,
                            unsigned vibration_hz)
{
    if (!motor_ready) return false;
    const esp_err_t err =
        drv8833_start_with_frequency(duration_ms, level, vibration_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start motor vibration: %s",
                 esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Motor vibration started: amplitude %u, %u Hz, duration %u ms.",
             level, vibration_hz, duration_ms);
    return true;
}

/* Queue a vibration to start after delay_ms. Only one vibration may be
 * pending; a newer request replaces an older one. The main loop actually
 * starts it once the deadline passes so all motor work stays in that task. */
static void schedule_vibration(uint32_t delay_ms, uint32_t duration_ms,
                               unsigned level, unsigned vibration_hz)
{
    vibration_pending = true;
    vibration_fire_at_us = esp_timer_get_time() + (int64_t)delay_ms * 1000;
    pending_vibration_duration_ms = duration_ms;
    pending_vibration_level = level;
    pending_vibration_hz = vibration_hz;
}

static void cancel_pending_vibration(void)
{
    vibration_pending = false;
}

static void process_pending_vibration(void)
{
    if (!vibration_pending) return;
    if (esp_timer_get_time() < vibration_fire_at_us) return;
    vibration_pending = false;
    drv8833_stop();
    start_vibration(pending_vibration_duration_ms, pending_vibration_level,
                    pending_vibration_hz);
}

static void start_awkward_vibration_timeline(void)
{
    cancel_pending_vibration();
    if (motor_ready) {
        drv8833_stop();
    }
    awkward_timeline_frequency_hz = 0;
    awkward_vibration_timeline_active = true;
}

static unsigned awkward_vibration_frequency_at(uint32_t elapsed_ms)
{
    /* The long-pulse effect takes priority where the schedule overlaps. */
    if ((elapsed_ms >= 7389 && elapsed_ms < 8312) ||
        (elapsed_ms >= 16625 && elapsed_ms < 17928)) {
        return AWKWARD_LONG_VIBRATION_HZ;
    }

    const size_t pulse_count =
        sizeof(awkward_short_vibration_starts_ms) /
        sizeof(awkward_short_vibration_starts_ms[0]);
    for (size_t i = 0; i < pulse_count; ++i) {
        uint32_t duration_ms = AWKWARD_SHORT_VIBRATION_MS;
        if (i == AWKWARD_EXTENDED_PULSE_INDEX) {
            duration_ms = AWKWARD_EXTENDED_PULSE_MS;
        } else if (i + 1 == pulse_count) {
            duration_ms = AWKWARD_FINAL_VIBRATION_MS;
        }
        const uint32_t start_ms = awkward_short_vibration_starts_ms[i];
        if (elapsed_ms >= start_ms && elapsed_ms - start_ms < duration_ms) {
            return AWKWARD_SHORT_VIBRATION_HZ;
        }
    }
    return 0;
}

static void process_awkward_vibration_timeline(void)
{
    if (!awkward_vibration_timeline_active) {
        return;
    }
    if (!speaker_is_song_playing()) {
        if (awkward_timeline_frequency_hz != 0 && motor_ready) {
            drv8833_stop();
        }
        awkward_timeline_frequency_hz = 0;
        awkward_vibration_timeline_active = false;
        return;
    }

    uint32_t elapsed_ms;
    if (!speaker_get_song_elapsed_ms(&elapsed_ms)) {
        return;
    }

    const unsigned frequency_hz =
        awkward_vibration_frequency_at(elapsed_ms);
    if (frequency_hz == awkward_timeline_frequency_hz) {
        return;
    }

    if (awkward_timeline_frequency_hz != 0 && motor_ready) {
        drv8833_stop();
    }
    awkward_timeline_frequency_hz = frequency_hz;
    if (frequency_hz != 0 && motor_ready) {
        const esp_err_t err = drv8833_start_with_frequency(
            0, AWKWARD_TIMELINE_AMPLITUDE, frequency_hz);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not start scheduled vibration: %s",
                     esp_err_to_name(err));
        }
    }
}

static bool trigger_reminder_stage(uint8_t stage)
{
    static const speaker_mp3_clip_t clips[] = {
        SPEAKER_MP3_CLIP_SOFT,
        SPEAKER_MP3_CLIP_BLAST,
        SPEAKER_MP3_CLIP_AWKWARD,
    };
    static const char *names[] = {
        "level one", "level two", "level three",
    };

    if (stage < 1 || stage > 3) return false;
    if (!reminders_enabled) {
        ESP_LOGW(TAG, "Stage %" PRIu8 " reminder skipped because reminders are muted.",
                 stage);
        return false;
    }
    if (!play_clip(clips[stage - 1], names[stage - 1])) return false;

    if (stage == 2) {
        schedule_vibration(BLAST_VIBRATION_DELAY_MS,
                           FIRST_LEVEL_VIBRATION_MS, blast_amplitude,
                           blast_frequency_hz);
    } else if (stage == 3) {
        start_awkward_vibration_timeline();
    }
    return true;
}

static void reset_daily_stats(uint32_t day_key, bool include_current_session)
{
    memset(&daily_stats, 0, sizeof(daily_stats));
    daily_stats.day_key = day_key;
    if (session_active && include_current_session) {
        daily_stats.sitting_seconds = session_seconds;
    }
    daily_counted_session_seconds = session_active ? session_seconds : 0;
    last_persisted_sitting_seconds = daily_stats.sitting_seconds;
    if (save_daily_stats() != ESP_OK) {
        ESP_LOGE(TAG, "Daily statistics reset was not persisted.");
    }
}

static bool set_system_clock(uint32_t local_epoch_seconds)
{
    const struct timeval baseline = {
        .tv_sec = (time_t)local_epoch_seconds,
        .tv_usec = 0,
    };
    if (settimeofday(&baseline, NULL) != 0) {
        ESP_LOGE(TAG, "Could not set the ESP32 RTC-backed clock: errno=%d",
                 errno);
        return false;
    }

    clock_synced = true;
    clock_read_error_reported = false;
    const uint32_t day_key = local_epoch_seconds / 86400U;
    if (daily_stats.day_key != day_key) reset_daily_stats(day_key, true);
    ESP_LOGI(TAG, "RTC-backed system clock synchronized; day key=%" PRIu32,
             day_key);
    return true;
}

static void sync_local_clock(uint32_t local_epoch_seconds)
{
    if (!set_system_clock(local_epoch_seconds)) {
        return;
    }
    const esp_err_t err = pcf8563_write_local_epoch(local_epoch_seconds);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not update PCF8563 clock: %s",
                 esp_err_to_name(err));
    }
}

static bool read_system_clock_epoch(uint32_t *epoch_seconds)
{
    struct timeval current_time;
    if (gettimeofday(&current_time, NULL) != 0) {
        if (!clock_read_error_reported) {
            ESP_LOGE(TAG, "Could not read the ESP32 RTC-backed clock: errno=%d",
                     errno);
            clock_read_error_reported = true;
        }
        return false;
    }
    if (current_time.tv_sec < 0 ||
        (uint64_t)current_time.tv_sec > UINT32_MAX) {
        if (!clock_read_error_reported) {
            ESP_LOGE(TAG, "ESP32 RTC-backed clock is outside the supported range.");
            clock_read_error_reported = true;
        }
        return false;
    }

    clock_read_error_reported = false;
    *epoch_seconds = (uint32_t)current_time.tv_sec;
    return true;
}

static uint32_t now_epoch_seconds(void)
{
    uint32_t epoch_seconds;
    return clock_synced && read_system_clock_epoch(&epoch_seconds)
               ? epoch_seconds
               : 0;
}

static uint32_t now_uptime_seconds(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

static void save_session_checkpoint(const session_checkpoint_t *checkpoint)
{
    nvs_handle_t handle;
    if (nvs_open(SESSION_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;
    nvs_set_blob(handle, SESSION_CHECKPOINT_KEY, checkpoint,
                 sizeof(*checkpoint));
    nvs_commit(handle);
    nvs_close(handle);
}

static bool load_session_checkpoint(session_checkpoint_t *checkpoint)
{
    memset(checkpoint, 0, sizeof(*checkpoint));
    nvs_handle_t handle;
    if (nvs_open(SESSION_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t size = sizeof(*checkpoint);
    const esp_err_t err =
        nvs_get_blob(handle, SESSION_CHECKPOINT_KEY, checkpoint, &size);
    nvs_close(handle);
    return err == ESP_OK && size == sizeof(*checkpoint);
}

/* Push one record to a connected phone over the history characteristic. This
 * is a no-op while no client is connected, so it is safe to call from the
 * boot-time interruption path before NimBLE is up. */
static bool send_record_notification(const record_t *record)
{
    char payload[128];
    const int length = snprintf(payload, sizeof(payload),
        "{\"s\":%lu,\"t\":%u,\"o\":%u,\"e\":%lu,\"u\":%lu,\"d\":%lu,"
        "\"r\":%lu,\"x\":%u,\"l\":%u,\"k\":%lu,\"g\":%lu}",
        (unsigned long)record->seq, (unsigned)record->type,
        (unsigned)record->online,
        (unsigned long)record->epoch_seconds,
        (unsigned long)record->uptime_seconds,
        (unsigned long)record->duration_seconds,
        (unsigned long)record->reminders, (unsigned)record->responded,
        (unsigned)record->level, (unsigned long)record->session_seq,
        (unsigned long)record->reason);
    if (length <= 0 || (size_t)length >= sizeof(payload)) return false;
    ble_control_notify_history(payload);
    return true;
}

/* Called once at boot: if the previous run ended mid-session (power cut,
 * brownout, watchdog, ...), append an interruption record so the phone can
 * show the abnormal event when it reconnects. */
static void record_abnormal_interruption(const session_checkpoint_t *checkpoint)
{
    record_t last;
    bool open_in_log = false;
    if (record_log_last_seq() != 0 &&
        record_log_read_next(record_log_last_seq() - 1, &last)) {
        open_in_log = last.type == RECORD_SESSION_START ||
                      last.type == RECORD_REMINDER;
    }

    if (!checkpoint->active && !open_in_log) return;

    record_t record = {0};
    record.type = RECORD_INTERRUPT;
    record.online = ble_control_is_connected();
    record.epoch_seconds =
        checkpoint->active ? now_epoch_seconds() : last.epoch_seconds;
    record.uptime_seconds = now_uptime_seconds();
    record.duration_seconds =
        checkpoint->active ? checkpoint->seated_seconds : 0;
    record.session_seq =
        checkpoint->active ? checkpoint->session_seq : last.seq;
    record.reason = (uint32_t)esp_reset_reason();
    if (record_log_append(&record) == ESP_OK) {
        send_record_notification(&record);
        ESP_LOGW(TAG, "Abnormal interruption recorded (reset reason %lu).",
                 (unsigned long)record.reason);
    }
}

static void process_sync_step(void)
{
    if (!sync_active) return;

    for (int i = 0; i < SYNC_BATCH_SIZE && sync_active; ++i) {
        record_t record;
        if (!record_log_read_next(sync_next_seq - 1, &record)) {
            char done[48];
            const int length = snprintf(done, sizeof(done),
                                        "{\"s\":-1,\"done\":1,\"max\":%lu}",
                                        (unsigned long)record_log_last_seq());
            if (length > 0 && (size_t)length < sizeof(done)) {
                ble_control_notify_history(done);
            }
            sync_active = false;
            ESP_LOGI(TAG, "History sync complete.");
            break;
        }

        send_record_notification(&record);
        sync_next_seq = record.seq + 1;
    }
}

static void process_control_commands(bool pressure)
{
    control_command_t command;
    while (xQueueReceive(control_queue, &command, 0) == pdTRUE) {
        if (command.type == CONTROL_SETTINGS) {
            audio_settings = command.settings;
            settings_saved = save_settings(&audio_settings) == ESP_OK;
            ESP_LOGI(TAG, "Audio settings applied: volume=%u%%, reminders="
                          "%" PRIu32 "/%" PRIu32 "/%" PRIu32 " ms.",
                     audio_settings.volume_percent, audio_settings.delay_ms[0],
                     audio_settings.delay_ms[1], audio_settings.delay_ms[2]);
            if (speaker_is_ready()) {
                const esp_err_t err =
                    speaker_set_volume_percent(audio_settings.volume_percent);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Could not update speaker volume: %s",
                             esp_err_to_name(err));
                }
            }
        } else if (command.type == CONTROL_ENABLED) {
            reminders_enabled = command.enabled;
            if (!reminders_enabled && speaker_is_song_playing()) {
                const esp_err_t err = speaker_stop();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Could not stop muted playback: %s",
                             esp_err_to_name(err));
                }
            }
            reminders_saved = save_enabled(reminders_enabled) == ESP_OK;
            if (!reminders_saved) {
                ESP_LOGE(TAG, "Reminder switch changed but could not be persisted.");
            }
        } else if (command.type == CONTROL_REMINDER) {
            if (trigger_reminder_stage(command.reminder_stage)) {
                ESP_LOGI(TAG, "Manually triggered reminder stage %u.",
                         (unsigned)command.reminder_stage);
            }
        } else if (command.type == CONTROL_STOP_ALL) {
            cancel_pending_vibration();
            awkward_vibration_timeline_active = false;
            awkward_timeline_frequency_hz = 0;
            if (motor_ready) {
                drv8833_stop();
            }
            if (speaker_is_ready()) {
                const esp_err_t err = speaker_stop();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Could not stop speaker playback: %s",
                             esp_err_to_name(err));
                }
            }
            ESP_LOGI(TAG, "Speaker and motor outputs stopped.");
        } else if (command.type == CONTROL_CLOCK) {
            sync_local_clock(command.local_epoch_seconds);
        } else if (command.type == CONTROL_MOTOR_FREQUENCY) {
            blast_frequency_hz = command.blast_frequency_hz;
            motor_frequency_hz = command.motor_frequency_hz;
            blast_amplitude = command.blast_amplitude;
            motor_amplitude = command.motor_amplitude;
            motor_settings_saved =
                save_motor_settings(blast_frequency_hz, motor_frequency_hz,
                                    blast_amplitude, motor_amplitude) == ESP_OK;
            if (!motor_settings_saved) {
                ESP_LOGE(TAG, "Motor settings changed but could not be persisted.");
            }
        } else if (command.type == CONTROL_VIBRATE) {
            if (!motor_ready) {
                ESP_LOGE(TAG, "Manual vibration skipped: motor is not ready.");
            } else {
                awkward_vibration_timeline_active = false;
                awkward_timeline_frequency_hz = 0;
                cancel_pending_vibration();
                drv8833_stop();
                if (command.vibrate_duration_ms == 0) {
                    ESP_LOGI(TAG, "Manual vibration stopped.");
                } else {
                    const esp_err_t err = drv8833_start_with_frequency(
                        command.vibrate_duration_ms, command.vibrate_level,
                        command.vibrate_frequency_hz);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Could not start manual vibration: %s",
                                 esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG,
                                 "Manual vibration started: amplitude=%u, "
                                 "frequency=%u Hz, duration=%" PRIu32 " ms.",
                                 (unsigned)command.vibrate_level,
                                 (unsigned)command.vibrate_frequency_hz,
                                 command.vibrate_duration_ms);
                    }
                }
            }
        } else if (command.type == CONTROL_SYNC) {
            sync_active = true;
            sync_next_seq = command.sequence + 1;
            ESP_LOGI(TAG, "History sync requested from seq %" PRIu32 ".",
                     command.sequence);
        } else if (command.type == CONTROL_ACK) {
            if (record_log_ack(command.sequence) == ESP_OK) {
                ESP_LOGI(TAG, "History sync acknowledged through seq %" PRIu32 ".",
                         command.sequence);
            }
        } else if (command.type == CONTROL_CLEAR_LOG) {
            if (record_log_clear() == ESP_OK) {
                ESP_LOGI(TAG, "Usage records cleared.");
            } else {
                ESP_LOGE(TAG, "Could not clear usage records.");
            }
        }
        publish_status(pressure);
    }
}

static void start_session(void)
{
    session_active = true;
    session_seconds = 0;
    session_reminder_stage = 0;
    session_reminders = 0;
    daily_counted_session_seconds = 0;
    session_seated_milliseconds = 0;
    last_repeat_milliseconds = 0;
    leave_pending_since = 0;

    session_start_epoch = now_epoch_seconds();
    session_start_uptime = now_uptime_seconds();

    record_t record = {0};
    record.type = RECORD_SESSION_START;
    record.online = ble_control_is_connected();
    record.epoch_seconds = session_start_epoch;
    record.uptime_seconds = session_start_uptime;
    if (record_log_append(&record) == ESP_OK) {
        current_session_seq = record.seq;
        send_record_notification(&record);
        const session_checkpoint_t checkpoint = {
            .active = 1,
            .start_epoch = session_start_epoch,
            .start_uptime = session_start_uptime,
            .seated_seconds = 0,
            .session_seq = current_session_seq,
        };
        save_session_checkpoint(&checkpoint);
    } else {
        current_session_seq = 0;
        ESP_LOGE(TAG, "Session start was not persisted.");
    }
    ESP_LOGI(TAG, "Continuous sitting session started.");
}

static void finish_session(void)
{
    daily_stats.sessions++;
    daily_stats.last_session_seconds = session_seconds;
    daily_stats.last_session_reminders = session_reminders;
    daily_stats.last_session_responded = session_reminders > 0 ? 1 : 0;

    record_t record = {0};
    record.type = RECORD_SESSION_END;
    record.online = ble_control_is_connected();
    record.epoch_seconds = now_epoch_seconds();
    record.uptime_seconds = now_uptime_seconds();
    record.duration_seconds = session_seconds;
    record.reminders = session_reminders;
    record.responded = session_reminders > 0;
    record.session_seq = current_session_seq;
    if (record_log_append(&record) != ESP_OK) {
        ESP_LOGE(TAG, "Finished sitting session was not logged.");
    } else {
        send_record_notification(&record);
    }
    current_session_seq = 0;
    const session_checkpoint_t checkpoint = {0};
    save_session_checkpoint(&checkpoint);

    session_active = false;
    session_seconds = 0;
    session_reminder_stage = 0;
    session_reminders = 0;
    session_seated_milliseconds = 0;
    leave_pending_since = 0;
    if (save_daily_stats() != ESP_OK) {
        ESP_LOGE(TAG, "Finished sitting session was not persisted.");
    }
    last_persisted_sitting_seconds = daily_stats.sitting_seconds;
    ESP_LOGI(TAG, "Sitting session ended; duration=%" PRIu32
                  " s, reminders=%" PRIu32 ", response=%s.",
             daily_stats.last_session_seconds,
             daily_stats.last_session_reminders,
             daily_stats.last_session_responded ? "yes" : "no");
}

static void log_reminder(uint8_t level)
{
    record_t record = {0};
    record.type = RECORD_REMINDER;
    record.online = ble_control_is_connected();
    record.epoch_seconds = now_epoch_seconds();
    record.uptime_seconds = now_uptime_seconds();
    record.level = level;
    record.session_seq = current_session_seq;
    if (record_log_append(&record) != ESP_OK) {
        ESP_LOGE(TAG, "Reminder event was not logged.");
    } else {
        send_record_notification(&record);
    }
}

static void handle_due_reminders(void)
{
    const uint32_t thresholds[] = {
        audio_settings.delay_ms[0],
        audio_settings.delay_ms[1],
        audio_settings.delay_ms[2],
    };
    if (session_reminder_stage < 3) {
        const uint32_t level = session_reminder_stage;
        if (session_seated_milliseconds < thresholds[level]) return;
        if (reminders_enabled && speaker_is_song_playing()) return;
        ESP_LOGI(TAG, "Sitting reminder level %" PRIu32 " due: elapsed=%" PRIu64
                      " ms, configured=%" PRIu32 " ms.",
                 level + 1, session_seated_milliseconds, thresholds[level]);
        const bool played = trigger_reminder_stage((uint8_t)(level + 1));
        session_reminder_stage++;
        if (played) {
            session_reminders++;
            daily_stats.reminders++;
            log_reminder((uint8_t)(level + 1));
        }
        if (level == 2) last_repeat_milliseconds = session_seated_milliseconds;
        return;
    }

    if (session_seated_milliseconds - last_repeat_milliseconds <
        LEVEL_THREE_REPEAT_MS) return;
    if (reminders_enabled && speaker_is_song_playing()) return;
    const bool played = trigger_reminder_stage(3);
    if (played) {
        session_reminders++;
        daily_stats.reminders++;
        log_reminder(3);
    }
    last_repeat_milliseconds = session_seated_milliseconds;
}

static void process_sitting(bool pressure, TickType_t now)
{
    static uint32_t reported_seconds;
    const TickType_t elapsed_ticks = now - previous_tick;
    previous_tick = now;

    if (!session_active) {
        if (!pressure) {
            seat_pending_since = 0;
            return;
        }
        if (seat_pending_since == 0) seat_pending_since = now;
        if ((uint64_t)(now - seat_pending_since) * portTICK_PERIOD_MS <
            SEAT_CONFIRM_MS) return;
        seat_pending_since = 0;
        start_session();
        reported_seconds = 0;
        return;
    }

    if (!pressure) {
        if (leave_pending_since == 0) leave_pending_since = now;
        if ((uint64_t)(now - leave_pending_since) * portTICK_PERIOD_MS >
            LEAVE_CONFIRM_MS) {
            finish_session();
            reported_seconds = 0;
        }
        return;
    }
    leave_pending_since = 0;

    session_seated_milliseconds +=
        (uint64_t)elapsed_ticks * portTICK_PERIOD_MS;
    session_seconds = (uint32_t)(session_seated_milliseconds / 1000U);
    if (session_seconds > daily_counted_session_seconds) {
        daily_stats.sitting_seconds +=
            session_seconds - daily_counted_session_seconds;
        daily_counted_session_seconds = session_seconds;
    }

    handle_due_reminders();
    if (session_seconds != reported_seconds) {
        reported_seconds = session_seconds;
        publish_status(pressure);
    }
}

static void rollover_daily_stats(void)
{
    if (!clock_synced) return;
    uint32_t epoch_seconds;
    if (!read_system_clock_epoch(&epoch_seconds)) return;
    const uint32_t today = epoch_seconds / 86400U;
    if (daily_stats.day_key != today) reset_daily_stats(today, false);
}

void app_main(void)
{
    /* Modem sleep: the BLE controller sleeps its radio between connection
     * events so the link stays alive while the CPU keeps running at full speed
     * (160 MHz). CPU down-clocking and light sleep are disabled to keep the
     * UART console and software I2C timing simple and reliable. */
    const esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, /* 160 MHz */
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, /* stay at 160 MHz */
        .light_sleep_enable = false,                     /* modem sleep only */
    };
    const esp_err_t pm_err = esp_pm_configure(&pm_config);
    if (pm_err != ESP_OK) {
        ESP_LOGW(TAG, "Power management unavailable (%s); "
                      "the device will run at full speed.",
                 esp_err_to_name(pm_err));
    }

    const esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed; settings will not persist: %s",
                 esp_err_to_name(nvs_err));
    }
    settings_saved = nvs_err == ESP_OK && load_settings(&audio_settings);
    if (nvs_err != ESP_OK) audio_settings = DEFAULT_SETTINGS;
    reminders_enabled = nvs_err == ESP_OK ? load_enabled(&reminders_saved) : true;
    if (nvs_err != ESP_OK) reminders_saved = false;
    if (nvs_err == ESP_OK) load_daily_stats();
    motor_settings_saved = nvs_err == ESP_OK && load_motor_settings();

    esp_err_t rtc_err = pcf8563_init();
    if (rtc_err == ESP_OK) {
        uint32_t local_epoch_seconds;
        rtc_err = pcf8563_read_local_epoch(&local_epoch_seconds);
        if (rtc_err == ESP_OK) {
            if (set_system_clock(local_epoch_seconds)) {
                ESP_LOGI(TAG, "System clock initialized from PCF8563 RTC.");
            }
        }
    }
    if (rtc_err != ESP_OK) {
        ESP_LOGW(TAG, "PCF8563 time unavailable; waiting for BLE time sync: %s",
                 esp_err_to_name(rtc_err));
    }

    if (nvs_err == ESP_OK && record_log_init() == ESP_OK) {
        session_checkpoint_t checkpoint;
        const bool had_checkpoint = load_session_checkpoint(&checkpoint);
        if (!had_checkpoint) memset(&checkpoint, 0, sizeof(checkpoint));
        record_abnormal_interruption(&checkpoint);
        const session_checkpoint_t cleared = {0};
        save_session_checkpoint(&cleared);
    } else if (nvs_err == ESP_OK) {
        ESP_LOGE(TAG, "Offline usage log unavailable; history will not be kept.");
    }

    ESP_ERROR_CHECK(pressure_sensor_init());
    const esp_err_t motor_err = drv8833_init();
    if (motor_err != ESP_OK) {
        ESP_LOGE(TAG, "Motor initialization failed: %s",
                 esp_err_to_name(motor_err));
    } else {
        motor_ready = true;
        start_vibration(STARTUP_VIBRATION_MS, STARTUP_VIBRATION_AMPLITUDE,
                        STARTUP_VIBRATION_HZ);
    }

    const esp_err_t speaker_err = speaker_init();
    if (speaker_err != ESP_OK) {
        ESP_LOGE(TAG, "Speaker initialization failed: %s",
                 esp_err_to_name(speaker_err));
    } else {
        ESP_ERROR_CHECK(
            speaker_set_volume_percent(audio_settings.volume_percent));
        play_clip(SPEAKER_MP3_CLIP_SOFT, "startup sound");
    }

    control_queue = xQueueCreate(8, sizeof(control_command_t));
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Could not create BLE control queue.");
        return;
    }
    previous_tick = xTaskGetTickCount();
    publish_status(pressure_sensor_is_pressed());
    const esp_err_t ble_err = ble_control_init();
    if (ble_err != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth sitting control unavailable: %s",
                 esp_err_to_name(ble_err));
    }

    ESP_LOGI(TAG, "Sitting reminders ready; thresholds=%" PRIu32 "/%" PRIu32
                  "/%" PRIu32 " ms, enabled=%s.",
             audio_settings.delay_ms[0], audio_settings.delay_ms[1],
             audio_settings.delay_ms[2], reminders_enabled ? "yes" : "no");

    TickType_t last_stats_save = xTaskGetTickCount();
    TickType_t last_status = last_stats_save;
    bool previous_pressure = pressure_sensor_is_pressed();
    bool first_report = true;
    while (true) {
        pressure_sensor_update();
        const bool pressure = pressure_sensor_is_pressed();
        const TickType_t now = xTaskGetTickCount();
        process_control_commands(pressure);
        process_pending_vibration();
        process_sync_step();
        rollover_daily_stats();

        /* Status is pushed over BLE on meaningful changes and on a slow
         * heartbeat, not every second. Every notification wakes the radio and
         * shortens the modem-sleep window, so keep idle traffic minimal. */
        if (first_report || pressure != previous_pressure) {
            first_report = false;
            publish_status(pressure);
        }
        if (pressure && !session_active && seat_pending_since == 0) {
            seat_pending_since = now;
        } else if (!pressure && !session_active) {
            seat_pending_since = 0;
        }
        process_sitting(pressure, now);
        process_awkward_vibration_timeline();

        if ((uint32_t)(now - last_status) >= pdMS_TO_TICKS(STATUS_HEARTBEAT_MS)) {
            publish_status(pressure);
            last_status = now;
        }
        if ((uint32_t)(now - last_stats_save) >= pdMS_TO_TICKS(60000)) {
            if (daily_stats.sitting_seconds != last_persisted_sitting_seconds) {
                if (save_daily_stats() == ESP_OK) {
                    last_persisted_sitting_seconds = daily_stats.sitting_seconds;
                }
            }
            if (session_active) {
                const session_checkpoint_t checkpoint = {
                    .active = 1,
                    .start_epoch = session_start_epoch,
                    .start_uptime = session_start_uptime,
                    .seated_seconds = session_seconds,
                    .session_seq = current_session_seq,
                };
                save_session_checkpoint(&checkpoint);
            }
            last_stats_save = now;
        }
        previous_pressure = pressure;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
