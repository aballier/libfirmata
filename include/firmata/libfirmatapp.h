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

#ifndef __LIBFIRMATAPP_H
#define __LIBFIRMATAPP_H

extern "C" {
#include <firmata/libfirmata.h>
}

class FirmataConn {
public:
    FirmataConn(const char* devname, int baudrate);
    ~FirmataConn();

    const struct firmata_global_data *get_global_state();
    int put_global_state(const struct firmata_global_data *s);
    int add_callback(int code, void (*cb)(void* arg));
    void del_callback(int code, void (*cb)(void* arg));
    void get_protocol_version();
    void get_firmware_version();
    void analog_mapping_query();
    void capability_query();
    void pin_state_query(uint8_t pin);
    void report_analog_channel(uint8_t chan, uint8_t val);
    void report_digital_port(uint8_t port, uint8_t val);
    void set_pin_mode(uint8_t pin, uint8_t mode);
    int  set_pin_value(uint8_t pin, uint32_t value);
    void set_sampling_interval(uint16_t ms);
    int  i2c_read_write(uint16_t addr, uint8_t command, int tenbits, const uint8_t *data, ssize_t data_len);
    void i2c_config(uint16_t delay);
    void encoder_attach(uint8_t encoder, uint8_t pin_a, uint8_t pin_b);
    void encoder_report_position(uint8_t encoder);
    void encoder_report_positions();
    void encoder_reset_position(uint8_t encoder);
    void encoder_set_reporting(uint8_t value);
    void encoder_detach(uint8_t encoder);
    void set_servo_config(uint8_t pin, uint16_t minPulse, uint16_t maxPulse);

private:
    struct firmata_conn *c;
};

#endif
