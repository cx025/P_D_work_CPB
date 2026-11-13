#include "pcf8563.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define PCF8563_ADDRESS 0x51
#define PCF8563_REG_CONTROL_STATUS1 0x00
#define PCF8563_REG_SECONDS 0x02
#define PCF8563_STOP_BIT 0x20
#define PCF8563_VOLTAGE_LOW_BIT 0x80
#define PCF8563_CENTURY_BIT 0x80
#define PCF8563_TRANSFER_TIMEOUT_MS 100

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t device_handle;

esp_err_t pcf8563_init(void)
{
    if (device_handle != NULL) {
        return ESP_OK;
    }

    if (bus_handle == NULL) {
        const i2c_master_bus_config_t bus_config = {
            .i2c_port = 0,
            .sda_io_num = GPIO_NUM_20,
            .scl_io_num = GPIO_NUM_21,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
        if (err != ESP_OK) {
            return err;
        }
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8563_ADDRESS,
        .scl_speed_hz = 100000,
    };
    return i2c_master_bus_add_device(bus_handle, &device_config,
                                     &device_handle);
}

static esp_err_t read_registers(uint8_t first_register, uint8_t *data,
                                size_t length)
{
    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(device_handle, &first_register, 1, data,
                                       length, PCF8563_TRANSFER_TIMEOUT_MS);
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(device_handle, data, sizeof(data),
                               PCF8563_TRANSFER_TIMEOUT_MS);
}

static bool decode_bcd(uint8_t raw, uint8_t mask, uint8_t maximum,
                       uint8_t *value)
{
    const uint8_t bcd = raw & mask;
    const uint8_t ones = bcd & 0x0f;
    const uint8_t tens = (bcd >> 4) & 0x0f;
    if (ones > 9 || tens > 9) {
        return false;
    }
    const uint8_t decoded = tens * 10 + ones;
    if (decoded > maximum) {
        return false;
    }
    *value = decoded;
    return true;
}

static bool valid_date(int year, unsigned month, unsigned day)
{
    static const uint8_t days_per_month[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    };
    if (month < 1 || month > 12) {
        return false;
    }
    unsigned maximum_day = days_per_month[month - 1];
    if (month == 2 &&
        (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
        maximum_day = 29;
    }
    return day >= 1 && day <= maximum_day;
}

static int64_t days_from_civil(int year, unsigned month, unsigned day)
{
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned year_of_era = (unsigned)(year - era * 400);
    const unsigned adjusted_month = month > 2 ? month - 3 : month + 9;
    const unsigned day_of_year =
        (153 * adjusted_month + 2) / 5 + day - 1;
    const unsigned day_of_era =
        year_of_era * 365 + year_of_era / 4 - year_of_era / 100 +
        day_of_year;
    return (int64_t)era * 146097 + day_of_era - 719468;
}

static uint8_t encode_bcd(unsigned value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

esp_err_t pcf8563_read_local_epoch(uint32_t *epoch_seconds)
{
    if (epoch_seconds == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t registers[7];
    uint8_t seconds_after_read = 0;
    bool stable = false;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        esp_err_t err =
            read_registers(PCF8563_REG_SECONDS, registers, sizeof(registers));
        if (err != ESP_OK) {
            return err;
        }
        if ((registers[0] & PCF8563_VOLTAGE_LOW_BIT) != 0) {
            return ESP_ERR_INVALID_STATE;
        }
        err = read_registers(PCF8563_REG_SECONDS, &seconds_after_read, 1);
        if (err != ESP_OK) {
            return err;
        }
        if ((seconds_after_read & PCF8563_VOLTAGE_LOW_BIT) != 0) {
            return ESP_ERR_INVALID_STATE;
        }
        if ((registers[0] & 0x7f) == (seconds_after_read & 0x7f)) {
            stable = true;
            break;
        }
    }
    if (!stable) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint8_t year_in_century;
    if (!decode_bcd(registers[0], 0x7f, 59, &second) ||
        !decode_bcd(registers[1], 0x7f, 59, &minute) ||
        !decode_bcd(registers[2], 0x3f, 23, &hour) ||
        !decode_bcd(registers[3], 0x3f, 31, &day) ||
        !decode_bcd(registers[5], 0x1f, 12, &month) ||
        !decode_bcd(registers[6], 0xff, 99, &year_in_century)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const int year = ((registers[5] & PCF8563_CENTURY_BIT) != 0 ? 1900 : 2000)
                     + year_in_century;
    if (year < 1970 || !valid_date(year, month, day)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const int64_t days = days_from_civil(year, month, day);
    const int64_t epoch = days * 86400 + hour * 3600 + minute * 60 + second;
    if (epoch < 0 || epoch > UINT32_MAX) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *epoch_seconds = (uint32_t)epoch;
    return ESP_OK;
}

esp_err_t pcf8563_write_local_epoch(uint32_t epoch_seconds)
{
    const time_t timestamp = (time_t)epoch_seconds;
    struct tm calendar;
    if (gmtime_r(&timestamp, &calendar) == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const int year = calendar.tm_year + 1900;
    if (year < 2000 || year > 2099) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t time_registers[] = {
        PCF8563_REG_SECONDS,
        encode_bcd((unsigned)calendar.tm_sec),
        encode_bcd((unsigned)calendar.tm_min),
        encode_bcd((unsigned)calendar.tm_hour),
        encode_bcd((unsigned)calendar.tm_mday),
        encode_bcd((unsigned)calendar.tm_wday),
        encode_bcd((unsigned)calendar.tm_mon + 1),
        encode_bcd((unsigned)(year - 2000)),
    };
    esp_err_t err = i2c_master_transmit(device_handle, time_registers,
                                        sizeof(time_registers),
                                        PCF8563_TRANSFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t control_status1;
    err = read_registers(PCF8563_REG_CONTROL_STATUS1, &control_status1, 1);
    if (err != ESP_OK) {
        return err;
    }
    return write_register(PCF8563_REG_CONTROL_STATUS1,
                          control_status1 & (uint8_t)~PCF8563_STOP_BIT);
}
