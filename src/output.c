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

#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include "firmata/libfirmata.h"
#include "libfirmata_internal.h"

/*

protocol
========

Current version: 2.4.0

The intention of this protocol is allow as much of the microcontroller to be controlled as possible from the host computer. This protocol then was designed for the direct communication between a microcontroller and an software object on a host computer. The host software object should then provide an interface that makes sense in that environment.

The data communication format uses MIDI messages. It is not necessarily a MIDI device, first it uses a faster serial speed, and second, the messages don't always map the same.


Message Types
===

This protocol uses the [MIDI message format](http://www.midi.org/techspecs/midimessages.php), but does not use the whole protocol.
Most of the command mappings here will not be directly usable in terms of MIDI
controllers and synths. It should co-exist with MIDI without trouble and can be
parsed by standard MIDI interpreters. Just some of the message data is used
differently.


| type                | command | MIDI channel | first byte          | second byte     |
| ------------------- | ------- | ------------ | ------------------- | --------------- |
| analog I/O message  | 0xE0    | pin #        | LSB(bits 0-6)       | MSB(bits 7-13)  |
| digital I/O message | 0x90    | port         | LSB(bits 0-6)       | MSB(bits 7-13)  |
| report analog pin   | 0xC0    | pin #        | disable/enable(0/1) | - n/a -         |
| report digital port | 0xD0    | port         | disable/enable(0/1) | - n/a -         |
|                     |         |              |                     |                 |
| start sysex         | 0xF0    |              |                     |                 |
| set pin mode(I/O)   | 0xF4    |              | pin # (0-127)       | pin state(0=in) |
| sysex end           | 0xF7    |              |                     |                 |
| protocol version    | 0xF9    |              | major version       | minor version   |


Sysex-based commands (0x00 - 0x7F) are used for an extended command set.

| type                  | command | first byte          | second byte   | ...            |
| --------------------- | ------- | ------------------- | ------------- | -------------- |
| string                | 0x71    | char *string ...    |               |                |
| firmware name/version | 0x79    | major version       | minor version | char *name ... |


Sysex Message Format
===

The idea for SysEx is to have a second command space using the first byte after
the SysEx start byte. The key difference is that data can be of any size, rather
than just one or two bytes for standard MIDI messages.

Generic SysEx Message
```
0   START_SYSEX (0xF0) (MIDI System Exclusive)
1   sysex command (0x00-0x7F)
... between 0 and MAX_DATA_BYTES 7-bit bytes of arbitrary data
N   END_SYSEX (0xF7) (MIDI End of SysEx - EOX)
```
*/

/**
 * @defgroup communications Communications
 * @brief Communication functions implementations.
 * @{
 */
static void firmata_send_msg(struct firmata_conn *c, const uint8_t *ibuf, size_t len)
{
    int written = 0;
#ifdef FIRMATA_DUMP_MESSAGES
    int i;
    printf("sending message: ");
    for(i=0;i<len;i++)
        printf("%X", ibuf[i]);
    printf("\n");
#endif

    pthread_mutex_lock(&c->write_mutex);
    while(written < len)
    {
        int ret = write(c->fd, ibuf + written, len - written);
        if(ret < 0 && (errno == EAGAIN || errno == EINTR)) ret = 0;
        if(ret < 0) break;
        written += ret;
    }
    pthread_mutex_unlock(&c->write_mutex);
}

static void firmata_send_code(struct firmata_conn *c, uint8_t code)
{
    const uint8_t ibuf[] = { START_SYSEX, code, END_SYSEX };
    firmata_send_msg(c, ibuf, sizeof(ibuf));
}

/*
 * | type                | command | MIDI channel | first byte          | second byte     |
 * | ------------------- | ------- | ------------ | ------------------- | --------------- |
 * | set pin mode(I/O)   | 0xF4    |              | pin # (0-127)       | pin state(0=in) |
 *
 * Set pin mode
 * ```
 * 0  set digital pin mode (0xF4) (MIDI Undefined)
 * 1  set pin number (0-127)
 * 2  state (INPUT/OUTPUT/ANALOG/PWM/SERVO/I2C/ONEWIRE/STEPPER/ENCODER, 0/1/2/3/4/6/7/8/9)
 * ```
 */
void firmata_set_pin_mode(struct firmata_conn *c, uint8_t pin, uint8_t mode)
{
    const uint8_t ibuf[] = { SET_PIN_MODE, pin, mode };

    if(pin >= MAX_NUM_PINS)
    {
        fprintf(stderr, "%s: invalid pin number (%i)\n", __func__, pin);
        return;
    }
    pthread_mutex_lock(&c->info_mutex);
    if(c->global_data.pin_state[pin].mode != mode)
    {
        struct pin_state_msg ps;
        firmata_send_msg(c, ibuf, sizeof(ibuf));
        c->global_data.pin_state[pin].mode = mode;
        ps = c->global_data.pin_state[pin];
        pthread_mutex_unlock(&c->info_mutex);
        process_callbacks(c, PIN_STATE_RESPONSE, &ps);
    }
    else
        pthread_mutex_unlock(&c->info_mutex);
}

/*
 * Two byte digital data format, second nibble of byte 0 gives the port number (eg 0x92 is the third port, port 2)
 * ```
 * 0  digital data, 0x90-0x9F, (MIDI NoteOn, bud different data format)
 * 1  digital pins 0-6 bitmask
 * 2  digital pin 7 bitmask
 * ```
 */
static void firmata_set_digital_pin_value(struct firmata_conn *c, uint8_t pin, uint8_t value)
{
    uint8_t ibuf[3];
    int port_num = pin / 8;
    int port_val = 0;
    int i;

    for(i=0; i<8; i++)
    {
        int p = port_num * 8 + i;
        if(c->global_data.pin_state[p].mode == MODE_OUTPUT || c->global_data.pin_state[p].mode == MODE_INPUT) {
            if (c->global_data.pin_state[p].value) {
                port_val |= (1<<i);
            }
        }
    }

    ibuf[0] = DIGITAL_IO | port_num;
    ibuf[1] = port_val & 0x7F;
    ibuf[2] = (port_val >> 7) & 0x01;
    firmata_send_msg(c, ibuf, sizeof(ibuf));
}

static void firmata_set_analog_value(struct firmata_conn *c, uint8_t pin, uint32_t value)
{
    if(pin < (1 << 4) && value < (1U << 8))
    {
        /*
         * Analog 14-bit data format
         * ```
         * 0  analog pin, 0xE0-0xEF, (MIDI Pitch Wheel)
         * 1  analog least significant 7 bits
         * 2  analog most significant 7 bits
         * ```
         */
        const uint8_t buf[] = {
            ANALOG_IO | (pin & 0x0F),
            value & 0x7F,
            (value >> 7) & 0x01
        };
        firmata_send_msg(c, buf, sizeof(buf));
    }
    else
    {
        /*
         * Extended Analog
         * ---
         *
         *  As an alternative to the normal analog message, this extended version allows
         *  addressing beyond pin 15 and supports sending analog values with any number of
         *  bits. The number of data bits is inferred by the length of the message.
         *
         *  ```
         *  0  START_SYSEX              (0xF0)
         *  1  extended analog message  (0x6F)
         *  2  pin                      (0-127)
         *  3  bits 0-6                 (least significant byte)
         *  4  bits 7-13                (most significant byte)
         *  ... additionaly bytes may be sent if more bits are needed
         *  N  END_SYSEX                (0xF7)
         *  ```
         */
        uint8_t buf[20];
        int shift;
        int pos = 0;
        buf[pos++] = START_SYSEX;
        buf[pos++] = EXTENDED_ANALOG;
        buf[pos++] = (pin & 0x7F);
        buf[pos++] = (value & 0x7F);
        for(shift = 7; shift < 8 * sizeof(value) && value >= (1 << shift); shift += 7)
            buf[pos++] = ((value >> shift) & 0x7F);
        buf[pos++] = END_SYSEX;
        firmata_send_msg(c, buf, pos);
    }
}

int firmata_set_pin_value(struct firmata_conn *c, uint8_t pin, uint32_t value)
{
    struct pin_state_msg ps;
    if(pin >= MAX_NUM_PINS)
        return EINVAL;
    pthread_mutex_lock(&c->info_mutex);
    switch(c->global_data.pin_state[pin].mode)
    {
        case MODE_OUTPUT:
            c->global_data.pin_state[pin].value = (value & 0xFF);
            ps = c->global_data.pin_state[pin];
            pthread_mutex_unlock(&c->info_mutex);
            firmata_set_digital_pin_value(c, pin, value & 0xFF);
            break;
        case MODE_PWM:
        case MODE_SERVO:
            c->global_data.pin_state[pin].value = value;
            ps = c->global_data.pin_state[pin];
            pthread_mutex_unlock(&c->info_mutex);
            firmata_set_analog_value(c, pin, value);
            break;
        default:
            pthread_mutex_unlock(&c->info_mutex);
            return EINVAL;
    }
    process_callbacks(c, PIN_STATE_RESPONSE, &ps);
    return 0;
}

/*
 * Request version report
 * ```
 * 0  request version report (0xF9) (MIDI Undefined)
 * ```
 */
void firmata_get_protocol_version(struct firmata_conn *c)
{
    static const uint8_t buf[] = { VERSION_REPORT };
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * Query Firmware Name and Version
 * ---
 *
 *  The firmware name to be reported should be exactly the same as the name of the
 *  Firmata client file, minus the file extension. So for StandardFirmata.ino, the
 *  firmware name is: StandardFirmata.
 *
 *  Query firmware Name and Version
 *  ```
 *  0  START_SYSEX       (0xF0)
 *  1  queryFirmware     (0x79)
 *  2  END_SYSEX         (0xF7)
 *  ```
 */
void firmata_get_firmware_version(struct firmata_conn *c)
{
    firmata_send_code(c, REPORT_FIRMWARE);
}

/*
 * | type                | command | MIDI channel | first byte          | second byte     |
 * | ------------------- | ------- | ------------ | ------------------- | --------------- |
 * | report analog pin   | 0xC0    | pin #        | disable/enable(0/1) | - n/a -         |
 *
 * Toggle analogIn reporting by pin
 * ```
 * 0  toggle analogIn reporting (0xC0-0xCF) (MIDI Program Change)
 * 1  disable(0) / enable(non-zero)
 * ```
 * *As of Firmata 2.4.0, upon enabling an analog pin, the pin value should be reported to the client
 * application.*
 */
void firmata_report_analog_channel(struct firmata_conn *c, uint8_t chan, uint8_t val)
{
    const uint8_t ibuf[] = { REPORT_ANALOG_PIN | (chan & 0x0F), val };
    firmata_send_msg(c, ibuf, sizeof(ibuf));
}

/*
 * | type                | command | MIDI channel | first byte          | second byte     |
 * | ------------------- | ------- | ------------ | ------------------- | --------------- |
 * | report digital port | 0xD0    | port         | disable/enable(0/1) | - n/a -         |
 *
 * Toggle digital port reporting by port (second nibble of byte 0), eg 0xD1 is port 1 is pins 8 to 15
 * ```
 * 0  toggle digital port reporting (0xD0-0xDF) (MIDI Aftertouch)
 * 1  disable(0) / enable(non-zero)
 * ```
 * *As of Firmata 2.4.0, upon enabling a digital port, the port value should be reported to the client
 * application.*
 */
void firmata_report_digital_port(struct firmata_conn *c, uint8_t port, uint8_t val)
{
    const uint8_t ibuf[] = { REPORT_DIGITAL_PORT | (port & 0x0F), val };
    firmata_send_msg(c, ibuf, sizeof(ibuf));
}


/*
 * Analog Mapping Query
 * ---
 *
 *  Analog messages are numbered 0 to 15, which traditionally refer to the Arduino
 *  pins labeled A0, A1, A2, etc. However, these pis are actually configured using
 *  "normal" pin numbers in the pin mode message, and when those pins are used for
 *  non-analog functions. The analog mapping query provides the information about
 *  which pins (as used with Firmata's pin mode message) correspond to the analog
 *  channels.
 *
 *  Analog mapping query
 *  ```
 *  0  START_SYSEX              (0xF0)
 *  1  analog mapping query     (0x69)
 *  2  END_SYSEX                (0xF7)
 *  ```
 */
void firmata_analog_mapping_query(struct firmata_conn *c)
{
    firmata_send_code(c, ANALOG_MAPPING_QUERY);
}

/*
 *
 * Capability Query
 * ---
 *
 *  The capability query provides a list of all modes supported by each pin and
 *  the resolution used by each pin. Each pin has 2 bytes for each supported mode
 *  and a value of 127 to mark the end of that pin's data. The number of pins
 *  supported is inferred by the message length. The resolution information may be
 *  used to adapt to future implementation where PWM, analog input and others may
 *  have different values (such as 12 or 14 bit analog instead of 10-bit analog).
 *  *For some features such as i2c, the resolution information is less important so
 *  a value of 1 is used.*
 *
 *  Capabilities query
 *  ```
 *  0  START_SYSEX              (0xF0)
 *  1  capabilities query       (0x6B)
 *  2  END_SYSEX                (0xF7)
 *  ```
 */
void firmata_capability_query(struct firmata_conn *c)
{
    firmata_send_code(c, CAPABILITY_QUERY);
}

/*
 * Pin State Query
 * ---
 *
 *  The pin **state** is any data written to the pin (*it is important to note that
 *  pin state != pin value*). For output modes (digital output,
 *  PWM, and Servo), the state is any value that has been previously written to the
 *  pin. For input modes, typically the state is zero. However, for digital inputs,
 *  the state is the status of the pullup resistor.
 *
 *  The pin state query can also be used as a verification after sending pin modes
 *  or data messages.
 *
 *  Pin state query
 *  ```
 *  0  START_SYSEX              (0xF0)
 *  1  pin state query          (0x6D)
 *  huhu missing pin # ???
 *  2  END_SYSEX                (0xF7)
 *  ```
 */
void firmata_pin_state_query(struct firmata_conn *c, uint8_t pin)
{
    const uint8_t buf[] = { START_SYSEX, PIN_STATE_QUERY, pin, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * Servo config
 * minPulse and maxPulse are 14-bit unsigned integers
 * 0  START_SYSEX          (0xF0)
 * 1  SERVO_CONFIG         (0x70)
 * 2  pin number           (0-127)
 * 3  minPulse LSB         (0-6)
 * 4  minPulse MSB         (0-13)
 * 5  maxPulse LSB         (0-6)
 * 6  maxPulse MSB         (7-13)
 * 7  END_SYSEX            (0xF7)
 */
void firmata_set_servo_config(struct firmata_conn *c, uint8_t pin, uint16_t minPulse, uint16_t maxPulse)
{
    const uint8_t ibuf[] = {
        START_SYSEX,
        SERVO_CONFIG,
        pin,
        minPulse & 0x7F,
        (minPulse >> 7) & 0x7F,
        maxPulse & 0x7F,
        (maxPulse >> 7) & 0x7F,
        END_SYSEX
    };
    firmata_send_msg(c, ibuf, sizeof(ibuf));
}

/*
 * Sampling Interval
 * ---
 *
 *  The sampling interval sets how often analog data and i2c data is reported to the 
 *  client. The default for the arduino implementation is 19ms. This means that every
 *  19ms analog data will be reported and any i2c devices with read continuous mode 
 *  will be read.
 *  ```
 *  0  START_SYSEX        (0xF0)
 *  1  SAMPLING_INTERVAL  (0x7A)
 *  2  sampling interval on the millisecond time scale (LSB)
 *  3  sampling interval on the millisecond time scale (MSB)
 *  4  END_SYSEX (0xF7)
 *  ```
 */
void firmata_set_sampling_interval(struct firmata_conn *c, uint16_t ms)
{
    const uint8_t ibuf[] = {
        START_SYSEX,
        SAMPLING_INTERVAL,
        ( ms & 0x7F ),
        (( ms >> 7 ) & 0x7F),
        END_SYSEX
    };
    firmata_send_msg(c, ibuf, sizeof(ibuf));
}

/*
 * I2C read/write request
 * ```
 * 0  START_SYSEX (0xF0)
 * 1  I2C_REQUEST (0x76)
 * 2  slave address (LSB)
 * 3  slave address (MSB) + read/write and address mode bits
 *    {7: always 0} + {6: reserved} + {5: address mode, 1 means 10-bit mode} +
 *    {4-3: read/write, 00 => write, 01 => read once, 10 => read continuously, 11 => stop reading} +
 *    {2-0: slave address MSB in 10-bit mode, not used in 7-bit mode}
 * 4  data 0 (LSB)
 * 5  data 0 (MSB)
 * 6  data 1 (LSB)
 * 7  data 1 (MSB)
 * ...
 * n  END_SYSEX (0xF7)
 * ```
 *
 * A note about read/write modes (above). The ```read continuously``` mode indicates that
 * the firmware should continuously read the device at the rate specified by the
 * [sampling interval](https://github.com/firmata/protocol/blob/master/protocol.md). A firmware implementation should support read continuous mode
 * for several I2C devices simultaneously. Sending the ```stop reading``` command will
 * end read continuous mode for that particular device.
 */
int firmata_i2c_read_write(struct firmata_conn *c, uint16_t addr, uint8_t command, int tenbits, const uint8_t *data, ssize_t data_len)
{
    uint8_t buf[MAX_DATA_BYTES] = { 0 };
    int i, pos = 0;
    if(data_len >= ((MAX_DATA_BYTES/2) - 7))
        return EINVAL;
    buf[pos++] = START_SYSEX;
    buf[pos++] = I2C_REQUEST;
    buf[pos++] = addr & 0x7F;
    if(tenbits)
    {
        buf[pos] |= (1 << 5);
        buf[pos] |= ((addr >> 7) & 0x07);
    }
    buf[pos] |= (command & 0x03) << 3;
    pos++;
    for(i=0;i<data_len;i++)
    {
        buf[pos++] = buf[i]         & 0x7F;
        buf[pos++] = (buf[i] >> 7 ) & 0x01;
    }
    buf[pos++] = END_SYSEX;
    firmata_send_msg(c, buf, pos);
    return 0;
}

/*
 * I2C config
 * ```
 * 0  START_SYSEX (0xF0)
 * 1  I2C_CONFIG (0x78)
 * 2  Delay in microseconds (LSB) [optional]
 * 3  Delay in microseconds (MSB) [optional]
 * ... user defined for special cases, etc
 * n  END_SYSEX (0xF7)
 * ```
 *
 * The optional ```Delay``` is for I2C devices that require a delay between when the
 * register is written to and the data in that register can be read.
 */
void firmata_i2c_config(struct firmata_conn *c, uint16_t delay)
{
    const uint8_t buf[] = { START_SYSEX, I2C_CONFIG, delay & 0x7F, (delay >> 7 & 0x7F), END_SYSEX };
    firmata_send_msg(c, buf, sizeof(buf));
}


/*
 * ### Attach encoder query
 * Query :
 * -----------------------------------------------------
 * 0 START_SYSEX                (0xF0)
 * 1 ENCODER_DATA               (0x61)
 * 2 ENCODER_ATTACH             (0x00)
 * 3 encoder #                  ([0 - MAX_ENCODERS-1])
 * 4 pin A #                    (first pin) 
 * 5 pin B #                    (second pin)
 * 6 END_SYSEX                  (0xF7)
 * -----------------------------------------------------
 */
void firmata_encoder_attach(struct firmata_conn *c, uint8_t encoder, uint8_t pin_a, uint8_t pin_b)
{
    const uint8_t buf[] = { START_SYSEX, ENCODER_DATA, ENCODER_ATTACH, encoder, pin_a, pin_b, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * -----------------------------------------------------
 * 0 START_SYSEX                (0xF0)
 * 1 ENCODER_DATA               (0x61)
 * 2 ENCODER_REPORT_POSITION    (0x01)
 * 3 Encoder #                  ([0 - MAX_ENCODERS-1])
 * 4 END_SYSEX                  (0xF7)
 * -----------------------------------------------------
 */
void firmata_encoder_report_position(struct firmata_conn *c, uint8_t encoder)
{
    const uint8_t buf[] = { START_SYSEX, ENCODER_DATA, ENCODER_REPORT_POSITION, encoder, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * -----------------------------------------------------
 * 0 START_SYSEX                (0xF0)
 * 1 ENCODER_DATA               (0x61)
 * 2 ENCODER_REPORT_POSITIONS   (0x02)
 * 3 END_SYSEX                  (0xF7)
 * -----------------------------------------------------
 */
void firmata_encoder_report_positions(struct firmata_conn *c)
{
    const uint8_t buf[] = { START_SYSEX, ENCODER_DATA, ENCODER_REPORT_POSITIONS, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * ### Reset encoder position to zero 
 * Query
 *  -----------------------------------------------------
 * 0 START_SYSEX                (0xF0)
 * 1 ENCODER_DATA               (0x61)
 * 2 ENCODER_RESET_POSITION     (0x03)
 * 3 encoder #                  ([0 - MAX_ENCODERS-1])
 * 4 END_SYSEX                  (0xF7)
 * -----------------------------------------------------
 */
void firmata_encoder_reset_position(struct firmata_conn *c, uint8_t encoder)
{
    const uint8_t buf[] = { START_SYSEX, ENCODER_DATA, ENCODER_RESET_POSITION, encoder, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * ### Enable/disable reporting 
 * Query
 * -----------------------------------------------------
 * 0 START_SYSEX                (0xF0)
 * 1 ENCODER_DATA               (0x61)
 * 2 ENCODER_REPORT_AUTO        (0x04)
 * 3 enable                     (0x00 => false, true otherwise)
 * 4 END_SYSEX                  (0xF7)
 * -----------------------------------------------------
 */
void firmata_encoder_set_reporting(struct firmata_conn *c, uint8_t value)
{
    const uint8_t buf[] = { START_SYSEX, ENCODER_DATA, ENCODER_REPORT_AUTO, value, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/*
 * ### Detach encoder
 * Query
 * -----------------------------------------------------
 * 0 START_SYSEX                (0xF0)
 * 1 ENCODER_DATA               (0x61)
 * 2 ENCODER_DETACH             (0x05)
 * 3 encoder #                  ([0 - MAX_ENCODERS-1])
 * 4 END_SYSEX                  (0xF7)
 -----------------------------------------------------
 */
void firmata_encoder_detach(struct firmata_conn *c, uint8_t encoder)
{
    const uint8_t buf[] = { START_SYSEX, ENCODER_DATA, ENCODER_DETACH, encoder, END_SYSEX};
    firmata_send_msg(c, buf, sizeof(buf));
}

/**
 * @}
 */

/*
TODO:

OneWire
===

The idea is to configure Arduino Pins as OneWire Busmaster. The may be more than one pin configured for OneWire and there may be more than one device connected to such a pin.

Each one-wire-device has a unique identifier which is 8 bytes long and comes factory-programmed into the the device. To scan all devices connected to a pin configured for onewire a SEARCH-request message is sent. The response contains all addresses of devices found. Having the address of a device OneWire-command-messages may be sent to this device.

The actual commands executed on the OneWire-bus are 'reset', 'skip', 'select', 'read', 'delay' and 'write' All these commands may be executed with a single OneWire-command-message. The subcommand-byte contains these commands bit-encoded. The data required to execute each bus-command must only be included in the message when the corresponding bit is set.

The order of execution of bus commands is: 'reset'->'skip'->'select'->'write'->'read'->'delay' (remember: each of these steps is optional. Also some combinations don't make sense and in fact are mutual exclusive in terms of OneWire bus protocol, so you cannot run a 'skip' followed by a 'select') The delay is useful for OneWire-commands included into taskdata (see [Firmata-scheduler proposal](https://github.com/firmata/protocol/blob/add-onewire/scheduler.md)).

Some OneWire-devices require some time to carry out e.g. a a/d-conversion after receiving the appropriate command. Including a delay into a OneWire-message saves some bytes in the taskdata (in comparism to the inclusion of a 'delay_task' scheduler message). OneWire Read- and ReadReply messages are correlated using a correlationid (16bits). The reply contains the correlationid-value that was sent with the original request.


Added in Firmata 2.5 ([configurable Firmata](https://github.com/firmata/arduino/tree/configurable)).


### Example files: 
 * OneWire is include by default in [ConfigurableFirmata.ino](https://github.com/firmata/arduino/blob/configurable/examples/ConfigurableFirmata/ConfigurableFirmata.ino). 
 * [Example implementation](https://github.com/firmata/arduino/blob/configurable/utility/OneWireFirmata.cpp) as a configurable Firmata feature class.


### Compatible host implementations
 * [Arduino firmata (configurable branch)](https://github.com/firmata/arduino/tree/configurable)


### Compatible client librairies
 * [perl-firmata](https://github.com/ntruchsess/perl-firmata)
 * [node-firmata](https://github.com/jgautier/firmata/blob/master/lib/firmata.js)


### Protocol details

OneWire SEARCH request
```
0  START_SYSEX      (0xF0)
1  OneWire Command  (0x73)
2  search command   (0x40|0x44) 0x40 normal search for all devices on the bus
0x44 SEARCH_ALARMS request to find only those
devices that are in alarmed state.
3  pin              (0-127)
4  END_SYSEX        (0xF7)
```

OneWire SEARCH reply
```
0  START_SYSEX      (0xF0)
1  OneWire Command  (0x73)
2  search reply command (0x42|0x45) 0x42 normal search reply
0x45 reply to a SEARCH_ALARMS request
3  pin              (0-127)
4  bit 0-6   [optional] address bytes encoded using 8 times 7 bit for 7 bytes of 8 bit
5  bit 7-13  [optional] 1.address[0] = byte[0]    + byte[1]<<7 & 0x7F
6  bit 14-20 [optional] 1.address[1] = byte[1]>>1 + byte[2]<<6 & 0x7F
7  ....                 ...
11 bit 49-55            1.address[6] = byte[6]>>6 + byte[7]<<1 & 0x7F
12 bit 56-63            1.address[7] = byte[8]    + byte[9]<<7 & 0x7F
13 bit 64-69            2.address[0] = byte[9]>>1 + byte[10]<<6 &0x7F
n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX      (0xF7)
```

OneWire CONFIG request
```
0  START_SYSEX      (0xF0)
1  OneWire Command  (0x73)
2  config command   (0x41)
3  pin              (0-127)
4  power            (0x00|0x01) 0x00 = leave pin on state high after write to support
parasitic power
    0x01 = don't leave pin on high after write
5  END_SYSEX (0xF7)
    ```

    OneWire COMMAND request
    ```
    0  START_SYSEX      (0xF0)
1  OneWire Command  (0x73)
    2  command bits     (0x00-0x2F) bit 0 = reset, bit 1 = skip, bit 2 = select,
    bit 3 = read, bit 4 = delay, bit 5 = write
3  pin              (0-127)
    4  bit 0-6   [optional] data bytes encoded using 8 times 7 bit for 7 bytes of 8 bit
    5  bit 7-13  [optional] data[0] = byte[0]   + byte[1]<<7 & 0x7F
    6  bit 14-20 [optional] data[1] = byte[1]>1 + byte[2]<<6 & 0x7F
    7  ....                 data[2] = byte = byte[2]>2 + byte[3]<<5 & 0x7F ...
    n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX      (0xF7)

    // data bytes within OneWire Request Command message
    0  address[0]                    [optional, if bit 2 set]
    1  address[1]                              "
    2  address[2]                              "
    3  address[3]                              "
    4  address[4]                              "
    5  address[5]                              "
    6  address[6]                              "
    7  address[7]                              "
    8  number of bytes to read (LSB) [optional, if bit 3 set]
    9  number of bytes to read (MSB)           "
    10 request correlationid byte 0            "
    11 request correlationid byte 1            "
    10 delay in ms      (bits 0-7)   [optional, if bit 4 set]
    11 delay in ms      (bits 8-15)            "
    12 delay in ms      (bits 16-23)           "
    13 delay in ms      (bits 24-31)           "
    14 data to write    (bits 0-7)   [optional, if bit 5 set]
    15 data to write    (bits 8-15)            "
    16 data to write    (bits 16-23)           "
    n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
    ```

    OneWire READ reply
    ```
    0  START_SYSEX          (0xF0)
    1  OneWire Command      (0x73)
    2  read reply command   (0x43)
3  pin                  (0-127)
    4  bit 0-6   [optional] data bytes encoded using 8 times 7 bit for 7 bytes of 8 bit
    5  bit 7-13  [optional] correlationid[0] = byte[0]   + byte[1]<<7 & 0x7F
    6  bit 14-20 [optional] correlationid[1] = byte[1]>1 + byte[2]<<6 & 0x7F
    7  bit 21-27 [optional] data[0] = byte[2]>2 + byte[3]<<5 & 0x7F
    8  ....                 data[1] = byte[3]>3 + byte[4]<<4 & 0x7F
    n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX          (0xF7)
    ```
    */

    /*
       TODO:

       Scheduler
       ===

       The idea is to store a stream of messages on a microcontroller which is replayed later (either once or repeated). A task is created by sending a create_task message. The time-to-run is initialized with 0 (which means the task is not yet ready to run). After filling up the taskdata with messages (using add_to_task command messages) a final schedule_task request is send, that sets the time-to-run (in milliseconds after 'now'). If a task itself contains delay_task or schedule_task-messages these cause the execution of the task to pause and resume after the amount of time given in such message has elapsed. If the last message in a taks is a delay_task message the task is scheduled for reexecution after the amount of time specified. If there's no delay_task message at the end of the task (so the time-to-run is not updated during the run) the task gets deleted after execution.


       Added in Firmata 2.5 ([configurable Firmata](https://github.com/firmata/arduino/tree/configurable)).


### Example files: 
     * OneWire is include by default in [ConfigurableFirmata.ino](https://github.com/firmata/arduino/blob/configurable/examples/ConfigurableFirmata/ConfigurableFirmata.ino). 
     * [Example implementation](https://github.com/firmata/arduino/blob/configurable/utility/FirmataScheduler.cpp) as a configurable Firmata feature class.


### Compatible host implementations
     * [Arduino firmata (configurable branch)](https://github.com/firmata/arduino/tree/configurable)


### Compatible client librairies
     * [perl-firmata](https://github.com/ntruchsess/perl-firmata)


### Protocol details

Scheduler CREATE_TASK request
```
0  START_SYSEX          (0xF0)
1  Scheduler Command    (0x7B)
2  create_task command  (0x00)
3  task id              (0-127)
4  length LSB           (bit 0-6)
5  length MSB           (bit 7-13)
6  END_SYSEX            (0xF7)
```

Scheduler DELETE_TASK request
```
0  START_SYSEX          (0xF0)
1  Scheduler Command    (0x7B)
2  delete_task command  (0x01)
3  task id              (0-127)
4  END_SYSEX            (0xF7)
```

Scheduler ADD_TO_TASK request
```
0  START_SYSEX          (0xF0)
1  Scheduler Command    (0x7B)
2  add_to_task command  (0x02)
3  task id              (0-127)
4  taskdata bit 0-6     [optional] task bytes encoded using 8 times 7 bit
for 7 bytes of 8 bit
5  taskdata bit 7-13    [optional]
6  taskdata bit 14-20   [optional]
n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX          (0xF7)
```

Scheduler DELAY_TASK request
```
0  START_SYSEX          (0xF0)
1  Scheduler Command    (0x7B)
2  delay_task command   (0x03)
3  time_ms bit 0-6      time_ms is of type long, requires 32 bit.
4  time_ms bit 7-13
5  time_ms bit 14-20
6  time_ms bit 21-27
7  time_ms bit 28-31
8  END_SYSEX            (0xF7)
    ```

    Scheduler SCHEDULE_TASK request
    ```
    0  START_SYSEX              (0xF0)
    1  Scheduler Command        (0x7B)
    2  schedule_task command    (0x04)
3  task id                  (0-127)
    4  time_ms bit 0-6          time_ms is of type long, requires 32 bit.
    5  time_ms bit 7-13
    6  time_ms bit 14-20
    7  time_ms bit 21-27
    8  time_ms bit 28-31
9  END_SYSEX                (0xF7)
    ```

    Scheduler QUERY_ALL_TASKS request
    ```
    0  START_SYSEX              (0xF0)
    1  Scheduler Command        (0x7B)
    2  query_all_tasks command  (0x05)
3  END_SYSEX                (0xF7)
    ```

    Scheduler QUERY_ALL_TASKS reply
    ```
    0  START_SYSEX          (0xF0)
    1  Scheduler Command    (0x7B)
2  query_all_tasks Reply Command (0x09)
    3  taskid_1             (0-127) [optional]
    4  taskid_2             (0-127) [optional]
    n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX (0xF7)
    ```

    Scheduler QUERY_TASK request
    ```
    0  START_SYSEX              (0xF0)
    1  Scheduler Command        (0x7B)
    2  query_task command       (0x06)
    3  task id                  (0-127)
4  END_SYSEX                (0xF7)
    ```

    Scheduler QUERY_TASK reply
    ```
    0  START_SYSEX          (0xF0)
    1  Scheduler Command    (0x7B)
    2  query_task Reply Commandc (0x0A)
3  task id              (0-127)
    4  time_ms bit 0-6
    5  time_ms bit 7-13
    6  time_ms bit 14-20
    7  time_ms bit 21-27
    8  time_ms bit 28-31 | (length bit 0-2) << 4
    9  length bit 3-9
    10 length bit 10-15 | (position bit 0) << 7
    11 position bit 1-7
    12 position bit 8-14
    13 position bit 15 | taskdata bit 0-5 << 1 [taskdata is optional]
    14 taskdata bit 6-12  [optional]
    15 taskdata bit 13-19 [optional]
    n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX          (0xF7)
    ```

    Scheduler RESET request
    ```
    0  START_SYSEX              (0xF0)
    1  Scheduler Command        (0x7B)
    2  scheduler reset command  (0x07)
3  END_SYSEX                (0xF7)
    ```

    Scheduler ERROR_FIRMATA_TASK reply
    ```
    0  START_SYSEX              (0xF0)
    1  Scheduler Command        (0x7B)
    2  error_task Reply Command (0x08)
3  task id                  (0-127)
    4  time_ms bit 0-6
    5  time_ms bit 7-13
    6  time_ms bit 14-20
    7  time_ms bit 21-27
    8  time_ms bit 28-31 | (length bit 0-2) << 4
    9  length bit 3-9
    10 length bit 10-15 | (position bit 0) << 7
    11 position bit 1-7
    12 position bit 8-14
    13 position bit 15 | taskdata bit 0-5 << 1 [taskdata is optional]
    14 taskdata bit 6-12  [optional]
    15 taskdata bit 13-19 [optional]
    n  ... as many bytes as needed (don't exceed MAX_DATA_BYTES though)
n+1  END_SYSEX              (0xF7)
    ```
    */

    /*
       TODO:

       Stepper Motor
       ===

       Provides support for 4 wire and 2 wire stepper motor drivers (H-bridge, darlington array, etc) as well as step + direction drivers such as the [EasyDriver](http://www.schmalzhaus.com/EasyDriver/).
       Current implementation supports 6 stepper motors at the same time (#[0-5]).

       Also includes optional support for acceleration and deceleration of the motor.

       Added in Firmata 2.5 ([configurable Firmata](https://github.com/firmata/arduino/tree/configurable)).

       Example files:
     * The Stepper feature is include by default in [ConfigurableFirmata.ino](https://github.com/firmata/arduino/blob/configurable/examples/ConfigurableFirmata/ConfigurableFirmata.ino).
     * [Example implementation](https://github.com/firmata/arduino/blob/configurable/utility/StepperFirmata.cpp) as a configurable Firmata feature class.
     * [Example of Stepper implementation in StandardFirmata](https://github.com/soundanalogous/AdvancedFirmata). *Note the dependency on the FirmataStepper class.*

     Protocol
     ---

     Stepper configuration

     *Note: `stepDelay` is the the number of microseconds between steps. The default
     value is 1us. You can change the delay to 2us (useful for high current stepper
     motor drivers). Additional delay values can be added in the future.*
     ```
     0  START_SYSEX                       (0xF0)
     1  Stepper Command                   (0x72)
     2  config command                    (0x00 = config, 0x01 = step)
     3  device number                     (0-5) (supports up to 6 motors)
     4  stepDelay | interface             (upper 4 bits = step delay:
     0000XXX = default 1us delay [default]
     0001XXX = 2us delay
     additional bits not yet used)

     (lower 3 bits = interface:
     XXXX001 = step + direction driver
     XXXX010 = two wire
     XXXX100 = four wire)
     5  steps-per-revolution LSB
     6  steps-per-revolution MSB
     7  motorPin1 or directionPin number  (0-127)
     8  motorPin2 or stepPin number       (0-127)
     9  [only when interface = 0x04] motorPin3 (0-127)
     10 [only when interface = 0x04] motorPin4 (0-127)
     11 END_SYSEX                         (0xF7)
     ```

     Stepper step
     ```
     0  START_SYSEX          (0xF0)
     1  Stepper Command      (0x72)
     2  config command       (0x01)
     3  device number        (0-5)
     4  direction            (0-1) (0x00 = CW, 0x01 = CCW)
     5  num steps byte1 LSB
     6  num steps byte2
     7  num steps byte3 MSB  (21 bits (2,097,151 steps max))
     8  speed LSB            (steps in 0.01*rad/sec  (2050 = 20.50 rad/sec))
     9  speed MSB
     10 [optional] accel LSB (acceleration in 0.01*rad/sec^2 (1000 = 10.0 rad/sec^2))
     11 [optional] accel MSB
     12 [optional] decel LSB (deceleration in 0.01*rad/sec^2)
     13 [optional] decel MSB
     14 END_SYSEX            (0xF7)
     ```
     */
