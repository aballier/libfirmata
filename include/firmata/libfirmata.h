/*
 * Copyright (c) 2015, Alexis Ballier.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the <ORGANIZATION> nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LIBFIRMATA_H
#define __LIBFIRMATA_H

#include <sys/types.h>
#include <inttypes.h>

#include "firmata/protocol.h"

/**
 * @defgroup limits Constant limits
 * @brief Constant limits
 * @{
 */

/**
 * @brief
 * Maximum number of pins supported.
 */
#define MAX_NUM_PINS	128

/**
 * @brief Maximum size of incoming messages.
 */
#define MAX_DATA_BYTES          256

/** @} */

/**
 * @defgroup msgs Message types
 * @brief Firmata messages
 * @{
 */

/**
 * @brief
 * Header common to all firmata messages.
 */
#define COMMON_HEADER \
    uint8_t start_code;\
uint8_t msg_type

/**
 * @brief
 * Trailer common to all firmata messages.
 */
#define COMMON_TRAILER \
    ssize_t size

/**
 * @brief
 * Generic firmata message.
 */
struct firmata_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    uint8_t data[MAX_DATA_BYTES]; /*!< Message data */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/**
 * @brief
 * Firmware information message.
 *
 * Passed as argument to #REPORT_FIRMWARE callbacks.
 */
struct firmware_info_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    uint8_t major; /*!< Major version */
    uint8_t minor; /*!< Minor version */
    char	name[MAX_DATA_BYTES]; /*!< Firmware name */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/**
 * @brief
 * Capabilities of a single pin.
 */
struct pin_cap {
    uint64_t supported_modes; /*!< Bitmask representing the supported modes */
    uint8_t resolution[MODE_MAX]; /*!< Resolution for each mode */
};

/**
 * @brief
 * Pin capabilities message.
 *
 * Passed as argument to #CAPABILITY_RESPONSE callbacks.
 */
struct pin_cap_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    struct pin_cap pin_info[MAX_NUM_PINS]; /*!< Capabilities of each pin */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/**
 * @brief
 * Analog mapping message.
 *
 * Passed as argument to #ANALOG_MAPPING_RESPONSE callbacks.
 */
struct analog_map_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    uint8_t analog_channel[MAX_NUM_PINS]; /*!< Analog channel corresponding to each pin */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/**
 * @brief
 * Pin state message.
 *
 * Passed as argument to #PIN_STATE_RESPONSE callbacks.
 */
struct pin_state_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    uint8_t num; /*!< Pin number */
    uint8_t mode; /*!< Pin mode */
    uint32_t value; /*!< Pin value */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/**
 * @brief
 * Protocol version message.
 *
 * Passed as argument to #VERSION_REPORT callbacks.
 */
struct protocol_version_msg {
    uint8_t start_code; /*!< Message start code (to be ignored) */
    uint8_t major; /*!< Major version */
    uint8_t minor; /*!< Minor version */
};

/**
 * @brief
 * Value of an encoder.
 */
struct encoder_value {
    uint8_t direction; /*!< Direction */
    uint32_t position; /*!< Position */
};

/**
 * @brief
 * Encoder values message.
 *
 * Passed as argument to #ENCODER_DATA callbacks.
 */
struct encoder_values_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    struct encoder_value encoders[MAX_ENCODERS]; /*!< Value of each encoder */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/**
 * @brief
 * I2C reply message.
 *
 * Passed as argument to #I2C_REPLY callbacks.
 */
struct firmata_i2c_msg {
    COMMON_HEADER; /*!< Header common to all messages */
    uint16_t addr; /*!< I2C address from whom the message comes */
    uint16_t reg; /*!< Register it comes from */
    ssize_t len; /*!< Length of the data */
    uint8_t data[MAX_DATA_BYTES]; /*!< Data */
    COMMON_TRAILER; /*!< Trailer common to all messages */
};

/** @} */

/**
 * @defgroup globalstate Global state
 * @brief Global state of a firmata device.
 * @{
 */

/**
 * @brief Global state of the firmata device.
 */
struct firmata_global_data {
    struct protocol_version_msg protocol_version;        /*!< Protocol version */
    struct firmware_info_msg    fw;                      /*!< Firmware information */
    struct pin_cap_msg          pin_cap;                 /*!< Pin capabilities */
    struct analog_map_msg       analog_map;              /*!< Analog map */
    struct pin_state_msg        pin_state[MAX_NUM_PINS]; /*!< Pin states */
    struct encoder_values_msg   encoders;                /*!< Encoder states */
};

/** @} */

/** 
 * @brief Opaque structure representing a connection to a firmata device.
 */
struct firmata_conn;

/**
 * @defgroup basic Basic functions
 * @brief Basic functions to start and stop a connection to a firmata device.
 * @{
 */

/**
 * @brief
 * Opens a connection to a firmata device.
 *
 * @param[in] devname Path to the serial device connected to the firmata device.
 * @param[in] baudrate Baudrate.
 * @return A valid pointer to a struct firmata_conn, NULL in case of error.
 */
struct firmata_conn *firmata_open(const char* devname, int baudrate);

/**
 * @brief
 * Closes a connection to a firmata device.
 *
 * @param[in] c Pointer to the firmata_conn to close. Must not be used after calling firmata_close.
 */
void firmata_close(struct firmata_conn *c);

/** @} */

/**
 * @defgroup callbacks Callback handling
 * @brief Register and de-register on given callbacks.
 * @{
 */

/**
 * @brief Registers on a callback type.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 * @param[in] code Callback code.
 * @param[in] cb Function to call.
 * @return 0 in case of success, a standard error code otherwise.
 */
int firmata_add_callback(struct firmata_conn *c, int code, void (*cb)(void* arg));

/**
 * @brief Deregisters on a callback type.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 * @param[in] code Callback code.
 * @param[in] cb Function to de-register.
 */
void firmata_del_callback(struct firmata_conn *c, int code, void (*cb)(void* arg));

/** @} */

/**
 * @defgroup desc Descriptions
 * @brief Get descriptions for various firmata fields.
 * @{
 */

/**
 * @brief Get the description of a message type.
 *
 * @param[in] msg A generic firmata message.
 * @return A pointer to a constant string representing the message type. NULL if not found.
 */
const char* firmata_get_msg_type_desc(struct firmata_msg* msg);

/**
 * @brief Get the description of the mode of a given pin.
 * 
 * @param[in] mode Pin mode.
 * @return A pointer to a constant string representing the mode. NULL if not found.
 */
const char* firmata_get_pin_mode_desc(uint8_t mode);

/**
 * @brief Fill mode descriptions from a supported mode mask.
 *
 * @param[in] supported_modes Supported mode mask.
 * @param[inout] values A pre-allocated array that is filled with pointers to constant strings describing each supported mode.
 * @param[in] len The length of values.
 * @return The number of filled modes.
 */
ssize_t firmata_fill_supported_modes(uint64_t supported_modes, const char** values, ssize_t len);

/** @} */

/**
 * @defgroup query Query
 * @brief Query values from the firmata device.
 * @{
 */

/**
 * @brief Request protocol version.
 *
 * When the response is received, a #VERSION_REPORT callback is generated.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 */
void firmata_get_protocol_version(struct firmata_conn *c);

/**
 * @brief Request firmware version.
 *
 * When the response is received, a #REPORT_FIRMWARE callback is generated.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 */
void firmata_get_firmware_version(struct firmata_conn *c);

/**
 * @brief Request analog mapping.
 *
 * When the response is received, a #ANALOG_MAPPING_RESPONSE callback is generated.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 */
void firmata_analog_mapping_query(struct firmata_conn *c);


/**
 * @brief Request pin capabilities.
 *
 * When the response is received, a #CAPABILITY_RESPONSE callback is generated.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 */
void firmata_capability_query(struct firmata_conn *c);

/**
 * @brief Request pin state.
 *
 * When the response is received, a #PIN_STATE_RESPONSE callback is generated.
 *
 * @param[in] c Pointer to the corresponding firmata_conn.
 * @param[in] pin Pin number.
 */
void firmata_pin_state_query(struct firmata_conn *c, uint8_t pin);

/** @} */


/**
 * @defgroup settings  Settings
 * @brief Set various values on the firmata device.
 * @{
 */

/**
 * @brief Toggle analog reporting by pin.
 *
 * Reporting generates #PIN_STATE_RESPONSE callbacks every 19ms for the default arduino implementation.
 * The interval can be changed with firmata_set_sampling_interval().
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] chan Channel to toggle.
 * @param[in] val Value: 1 for reporting, 0 for not reporting.
 */
void firmata_report_analog_channel(struct firmata_conn *c, uint8_t chan, uint8_t val);

/**
 * @brief Toggle digital port reporting.
 *
 * Reporting generates #PIN_STATE_RESPONSE callbacks every 19ms for the default arduino implementation.
 * The interval can be changed with firmata_set_sampling_interval().
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] port Port to toggle.
 * @param[in] val Value: 1 for reporting, 0 for not reporting.
 */
void firmata_report_digital_port(struct firmata_conn *c, uint8_t port, uint8_t val);

/**
 * @brief Set the mode of a pin.
 *
 * A #PIN_STATE_RESPONSE callback is generated after a successful setting.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] pin Pin number.
 * @param[in] mode Mode to set. See @ref pinmodes for valid values.
 */
void firmata_set_pin_mode(struct firmata_conn *c, uint8_t pin, uint8_t mode);

/**
 * @brief Set value of a pin.
 *
 * A #PIN_STATE_RESPONSE callback is generated after a successful setting.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] pin Pin number.
 * @param[in] value Value to set. This can represent either an analog or a digital value, depending on the pin mode.
 */
int firmata_set_pin_value(struct firmata_conn *c, uint8_t pin, uint32_t value);

/**
 * @brief Set sampling/reporting interval.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] ms Sampling interval in milliseconds.
 */
void firmata_set_sampling_interval(struct firmata_conn *c, uint16_t ms);

/** @} */

/**
 * @defgroup i2c I2C
 * @brief I2C related functions.
 * @{
 */

/**
 * @brief Send an I2C command.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] addr Address to send on the I2C bus.
 * @param[in] command Command to send. See @ref i2ccom.
 * @param[in] tenbits Boolean to specify if using 10-bits addressing on the bus.
 * @param[in] data Data to send
 * @param[in] data_len Length of data
 * @return 0 in case of success, a value representing the error otherwise.
 */
int firmata_i2c_read_write(struct firmata_conn *c, uint16_t addr, uint8_t command, int tenbits, const uint8_t *data, ssize_t data_len);

/**
 * @brief Configure I2C.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] delay Delay
 */
void firmata_i2c_config(struct firmata_conn *c, uint16_t delay);

/** @} */

/**
 * @defgroup encoder Encoders
 * @brief Encoder related functions
 * @{
 */

/**
 * @brief Attach an encoder to two pins.
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] encoder Encoder number.
 * @param[in] pin_a First pin.
 * @param[in] pin_b Second pin.
 */
void firmata_encoder_attach(struct firmata_conn *c, uint8_t encoder, uint8_t pin_a, uint8_t pin_b);

/**
 * @brief Ask for a given encoder to report its position.
 *
 * The response generates an #ENCODER_DATA callback.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] encoder Encoder number.
 */
void firmata_encoder_report_position(struct firmata_conn *c, uint8_t encoder);

/**
 * @brief Asks for all encoders to report their positions.
 *
 * The response generates an #ENCODER_DATA callback.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 */
void firmata_encoder_report_positions(struct firmata_conn *c);

/**
 * @brief Reset encoder position to zero.
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] encoder Encoder number.
 */
void firmata_encoder_reset_position(struct firmata_conn *c, uint8_t encoder);

/**
 * @brief Toggle automatic reporting of encoders.
 *
 * Reporting generates #ENCODER_DATA callbacks every 19ms for the default arduino implementation.
 * The interval can be changed with firmata_set_sampling_interval().
 *
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] value Boolean to enable (1) or disable (0) automatic reporting.
 */
void firmata_encoder_set_reporting(struct firmata_conn *c, uint8_t value);

/**
 * @brief Detach an encoder.
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] encoder Encoder number.
 */
void firmata_encoder_detach(struct firmata_conn *c, uint8_t encoder);

/** @} */

/**
 * @defgroup servos Servos
 * @brief Servos related functions
 * @{
 */

/**
 * @brief Set servo configuration.
 * @param[in] c Pointer to the corresponding struct firmata_conn.
 * @param[in] pin Pin number.
 * @param[in] minPulse Minimum pulse.
 * @param[in] maxPulse Maximum pulse.
 */
void firmata_set_servo_config(struct firmata_conn *c, uint8_t pin, uint16_t minPulse, uint16_t maxPulse);

/* @} */

#endif
