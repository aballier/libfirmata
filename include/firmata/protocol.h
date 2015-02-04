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

#ifndef __FIRMATA_PROTOCOL_H
#define __FIRMATA_PROTOCOL_H

/**
 * @defgroup msgstart Message start codes
 * @brief Valid values for #COMMON_HEADER start_code.
 * @{
 */
#define START_SYSEX             0xF0 //!< Start a MIDI Sysex message
#define SET_PIN_MODE	        0xF4 //!< Set pin mode
#define END_SYSEX               0xF7 //!< End a MIDI Sysex message
#define VERSION_REPORT          0xF9 //!< Protocol version report
#define ANALOG_IO               0xE0 //!< Analog I/O message
#define DIGITAL_IO              0x90 //!< Digital I/O message
#define REPORT_ANALOG_PIN       0xC0 //!< Report analog pin
#define REPORT_DIGITAL_PORT     0xD0 //!< Report digital port

/** @} */

/**
 * @defgroup msgtype Message types
 * @brief Valid values for #COMMON_HEADER msg_type.
 * @{
 */
#define ENCODER_DATA                0x61 //!< Reply with encoders current positions
#define ANALOG_MAPPING_QUERY        0x69 //!< Ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE     0x6A //!< Reply with mapping info
#define CAPABILITY_QUERY            0x6B //!< Ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE         0x6C //!< Reply with supported modes and resolution
#define PIN_STATE_QUERY             0x6D //!< Ask for a pin's current mode and state
#define PIN_STATE_RESPONSE          0x6E //!< Reply with a pin's current mode and state
#define EXTENDED_ANALOG             0x6F //!< Analog write (PWM, Servo, etc) to a pin
#define SERVO_CONFIG                0x70 //!< Servo configuration
#define STRING_DATA                 0x71 //!< String message with 14-bits per char
#define STEPPER_DATA                0x72 //!< Stepper motor data
#define ONEWIRE_DATA                0x73 //!< OneWire read/write/reset/select/skip/search request
#define SHIFT_DATA                  0x75 //!< ShiftOut config/data message (reserved - not yet implemented)
#define I2C_REQUEST                 0x76 //!< I2C request messages from a host to an I/O board
#define I2C_REPLY                   0x77 //!< I2C reply messages from an I/O board to a host
#define I2C_CONFIG                  0X78 //!< Enable I2C and provide any configuration settings
#define REPORT_FIRMWARE             0x79 //!< Report name and version of the firmware
#define SAMPLING_INTERVAL      	    0x7A //!< Interval at which analog input is sampled (default = 19ms)
#define SCHEDULER_DATA              0x7B //!< Send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
#define SYSEX_NON_REALTIME          0x7E //!< MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME              0X7F //!< MIDI Reserved for realtime messages

/** @} */

/**
 * @defgroup pinmodes Pin modes
 * @brief Pin mode values.
 * @{
 */

#define MODE_INPUT    0x00  //!< Input mode
#define MODE_OUTPUT   0x01  //!< Output mode
#define MODE_ANALOG   0x02  //!< Analog mode
#define MODE_PWM      0x03  //!< PWM mode
#define MODE_SERVO    0x04  //!< Servo mode
#define MODE_SHIFT    0x05  //!< Shift mode (not implemented)
#define MODE_I2C      0x06  //!< I2C mode
#define MODE_ONEWIRE  0x07  //!< Onewire mode
#define MODE_STEPPER  0x08  //!< Stepper mode
#define MODE_ENCODER  0x09  //!< Encoder mode
#define MODE_MAX      ( MODE_ENCODER + 1 )  //!< Valid modes limit. Not part of the ABI.

/** @} */

/**
 * @defgroup enccst Encoder constants
 * @brief Various constants for working with encoders.
 * @{
 */

/**
 * @brief Maximum number of encoders.
 */
#define MAX_ENCODERS	16

#define ENCODER_ATTACH             (0x00) /*!< Attach command */
#define ENCODER_REPORT_POSITION    (0x01) /*!< Report position */
#define ENCODER_REPORT_POSITIONS   (0x02) /*!< Report all positions */
#define ENCODER_RESET_POSITION     (0x03) /*!< Reset position */
#define ENCODER_REPORT_AUTO        (0x04) /*!< Auto reporting */
#define ENCODER_DETACH             (0x05) /*!< Detach command */

/** @} */

/**
 * @defgroup i2ccom I2C Read write commands
 * @brief Commands for the firmata_i2c_read_write() function.
 * @{
 */

/* read/write, 00 => write, 01 => read once, 10 => read continuously, 11 => stop reading} */
#define I2C_WRITE       0x00    //!< Write request
#define I2C_READ_ONCE   0x01    //!< Read once request
#define I2C_READ_CONT   0x02    //!< Read continuously request
#define I2C_STOP_READ   0x03    //!< Stop reading request

/* @} */

#endif // __FIRMATA_PROTOCOL_H
