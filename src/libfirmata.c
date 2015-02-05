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

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include <pthread.h>
#include <poll.h>


#include "firmata/libfirmata.h"
#include "firmata/protocol.h"
#include "libfirmata_internal.h"

void process_callbacks(struct firmata_conn *c, uint8_t code, void *arg)
{
    int i;
    pthread_mutex_lock(&c->cb_mutex);
    for(i=0;i<c->num_callbacks[code];i++)
    {
        void (*cb)(void* arg) = c->callbacks[code][i];
        pthread_mutex_unlock(&c->cb_mutex);
        cb(arg);
        pthread_mutex_lock(&c->cb_mutex);
    }
    pthread_mutex_unlock(&c->cb_mutex);
}

static inline int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
    switch(baudrate) {
        B(50);     B(75);     B(110);    B(134);    B(150);
        B(200);    B(300);    B(600);    B(1200);   B(1800);
        B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
        B(57600);  B(115200); B(230400); B(460800); B(500000);
        B(576000); B(921600); B(1000000);B(1152000);B(1500000);
        default: return 0;
    }
#undef B
}

static void get_message(int fd, struct firmata_msg *msg)
{
    ssize_t recvd = 0;
    int ret;
    int need_end_sysex = 0;
    int msg_len = 1;
    uint8_t t;
    uint8_t *buf = (uint8_t*)msg;

    ret = read(fd, ((uint8_t*)msg), 1);
    if(ret < 1) return;

    t = *buf & 0xF0;

    if(ANALOG_IO == t || DIGITAL_IO == t || VERSION_REPORT == *buf)
        msg_len = 2;
    else if(*buf == START_SYSEX)
        need_end_sysex = 1;
    else if(*buf & 0x80)
        msg_len = 0;

    recvd = 1;

    while(recvd < sizeof(*msg) && (need_end_sysex || msg_len > 0))
    {
        ret = read(fd, buf + recvd, msg_len);
        if(ret < 0)
        {
            perror("reading");
            sleep(1);
            continue;
        }
        recvd += ret;
        if(!need_end_sysex)
            msg_len -= ret;
        if(need_end_sysex && buf[recvd-1] == END_SYSEX)
        {
            need_end_sysex = 0;
            msg_len = 0;
        }
    }

    if(recvd >= (sizeof(*msg)))
        fprintf(stderr, "%s: Message too long\n", __func__);
    msg->size = recvd;
}

#define COPY_COMMON(src, dst) \
    dst->start_code = src->start_code;\
dst->msg_type = src->msg_type; \
dst->size = src->size

/*
 * Receive Firmware Name and Version (after query)
 * ```
 * 0  START_SYSEX       (0xF0)
 * 1  queryFirmware     (0x79)
 * 2  major version     (0-127)
 * 3  minor version     (0-127)
 * 4  first 7-bits of firmware name
 * 5  second 7-bits of firmware name
 * ... for as many bytes as it needs
 * N  END_SYSEX         (0xF7)
 * ```
 */
static inline void msg_to_firmware(const struct firmata_msg *msg, struct firmware_info_msg *fw)
{
    int i, pos = 0;
    COPY_COMMON(msg,fw);
    fw->major      = msg->data[0];
    fw->minor      = msg->data[1];
    for(i=2; i < msg->size - 4; i+=2)
        fw->name[pos++] = (msg->data[i] & 0x7F) | ((msg->data[i+1] & 0x7F) << 7);
    fw->name[pos] = 0;
}

/*
 * Capabilities response
 * ```
 * 0  START_SYSEX              (0xF0)
 * 1  capabilities response    (0x6C)
 * 2  1st mode supported of pin 0
 * 3  1st mode's resolution of pin 0
 * 4  2nd mode supported of pin 0
 * 5  2nd mode's resolution of pin 0
 * ... additional modes/resolutions, followed by a single 127 to mark the end of
 *     the pin's modes. Each pin follows with its mode and 127 until all pins are
 *         implemented.
 * N  END_SYSEX                 (0xF7)
 * ```
 */
static inline void msg_to_pin_info(const struct firmata_msg *msg, struct pin_cap_msg *pi)
{
    int pin, pos;

    COPY_COMMON(msg,pi);

    pos = 0;
    pin = 0;
    while(pos < msg->size - 2 && pin < MAX_NUM_PINS)
    {
        if(msg->data[pos] == 127)
        {
            /* next pin */
            pin++;
            pos++;
            continue;
        }

        pi->pin_info[pin].supported_modes |= (1 << msg->data[pos]);
        if(msg->data[pos] < MODE_MAX)
            pi->pin_info[pin].resolution[msg->data[pos]] = msg->data[pos+1];
        pos+=2;
    }
}

/*
 * Analog mapping response
 * ```
 * 0  START_SYSEX              (0xF0)
 * 1  analog mapping response  (0x6A)
 * 2  analog channel corresponding to pin 0, or 127 if pin 0 does not support analog
 * 3  analog channel corresponding to pin 1, or 127 if pin 1 does not support analog
 * 4  analog channel corresponding to pin 2, or 127 if pin 2 does not support analog
 * ... etc, one byte for each pin
 * N  END_SYSEX                (0xF7)
 * ```
 */
static inline void msg_to_analog_map(const struct firmata_msg *msg, struct analog_map_msg *am)
{
    int i;

    COPY_COMMON(msg,am);

    for(i=0 ; i < MAX_NUM_PINS; i++)
        am->analog_channel[i] = 127;

    for(i=0 ; i<msg->size-1; i++)
        am->analog_channel[i] = msg->data[i];
}

/*
 * Pin state response
 * ```
 * 0  START_SYSEX              (0xF0)
 * 1  pin state response       (0x6E)
 * 2  pin                      (0-127)
 * 3  pin mode (the currently configured mode)
 * 4  pin state, bits 0-6
 * 5  (optional) pin state, bits 7-13
 * 6  (optional) pin state, bits 14-20
 * ... additional optional bytes, as many as needed
 * N  END_SYSEX                (0xF7)
 * ```
 */
static inline void msg_to_pin_state(const struct firmata_msg *msg, struct pin_state_msg *ps)
{
    COPY_COMMON(msg,ps);

    if(msg->size < 6)
    {
        fprintf(stderr, "%s: Message is too short (%zu)\n", __func__, msg->size);
        return;
    }

    ps->num        = msg->data[0];
    ps->mode       = msg->data[1];
    ps->value      = msg->data[2];
    if(msg->size >= 7) ps->value |= (msg->data[3] << 7);
    if(msg->size >= 8) ps->value |= (msg->data[4] << 14);
}

/*
 * I2C reply
 * ```
 * 0  START_SYSEX (0xF0)
 * 1  I2C_REPLY (0x77)
 * 2  slave address (LSB)
 * 3  slave address (MSB)
 * 4  register (LSB)
 * 5  register (MSB)
 * 6  data 0 (LSB)
 * 7  data 0 (MSB)
 * ...
 * n  END_SYSEX (0XF7)
 * ```
 */
static inline void msg_to_i2c_msg(const struct firmata_msg *msg, struct firmata_i2c_msg *i)
{
    ssize_t c;
    COPY_COMMON(msg,i);
    i->addr = ( msg->data[0] & 0x7F) | (( msg->data[1] & 0x07 ) << 7);
    i->reg  = ( msg->data[2] & 0x7F) | (( msg->data[3] & 0x07 ) << 7);
    i->len  = 0;
    for(c=4; c < msg->size - 3 ; c+=2)
        i->data[i->len++] = ( msg->data[c] & 0x7F ) | (( msg->data[c+1] & 0x01 ) << 7);
}

/*
 *  -----------------------------------------------------
 *  0 START_SYSEX                (0xF0)
 *  1 ENCODER_DATA               (0x61)
 *  2 first enc. #  & first enc. dir.   [= (direction << 6) | (#)] 
 *  3 first enc. position, bits 0-6
 *  4 first enc. position, bits 7-13
 *  5 first enc. position, bits 14-20
 *  6 first enc. position, bits 21-27
 *  7 second enc. #  & second enc. dir. [= (direction << 6) | (#)]
 *  ...
 *  N END_SYSEX                  (0xF7)
 *  -----------------------------------------------------
 */
static inline void msg_to_enc_values(const struct firmata_msg *msg, struct encoder_values_msg *e)
{
    int i;
    COPY_COMMON(msg,e);
    for(i=0; (i < msg->size - 3 - 4); i+=5)
    {
        uint8_t enc = (msg->data[i] & 0x3F);
        if(enc >= MAX_ENCODERS)
        {
            fprintf(stderr, "%s: invalid encoder (%i)\n", __func__, enc);
            continue;
        }
        e->encoders[enc].direction = ((msg->data[i] >> 6) & 0x01);
        e->encoders[enc].position  = (
                (  msg->data[i+1] & 0x7F) |
                ( (msg->data[i+2] & 0x7F) << 7 ) |
                ( (msg->data[i+3] & 0x7F) << 14) |
                ( (msg->data[i+4] & 0x7F) << 21));
    }
}

static inline void ask_pin_info(struct pin_cap_msg *pi, struct firmata_conn *c)
{
    uint8_t pin;
    for(pin = 0 ; pin < MAX_NUM_PINS ; pin++)
    {
        if(!pi->pin_info[pin].supported_modes) continue;
        firmata_pin_state_query(c, pin);
    }
}

static void process_message(struct firmata_conn *c)
{
    struct firmata_msg msg         = { 0 };
    struct firmware_info_msg fw    = { 0 };
    struct pin_cap_msg pi          = { 0 };
    struct analog_map_msg am       = { 0 };
    struct pin_state_msg ps        = { 0 };
    struct firmata_i2c_msg i2c_msg = { 0 };
    struct encoder_values_msg ev;

    void *arg = &msg;
    uint8_t cmd;
    int i;

    get_message(c->fd, &msg);

    cmd = msg.start_code & 0xF0;

    switch(cmd)
    {
        case ANALOG_IO:
            if(msg.size >= 1)
            {
                /*
                 * Analog 14-bit data format
                 * ```
                 * 0  analog pin, 0xE0-0xEF, (MIDI Pitch Wheel)
                 * 1  analog least significant 7 bits
                 * 2  analog most significant 7 bits
                 * ```
                 */
                int pin;
                int analog_ch = (msg.start_code & 0x0F);
                int analog_val = msg.msg_type | (msg.data[0] << 7);
                for(pin = 0; pin < MAX_NUM_PINS; pin++)
                    if(c->global_data.analog_map.analog_channel[pin] == analog_ch)
                    {
                        struct pin_state_msg ps;
                        pthread_mutex_lock(&c->info_mutex);
                        c->global_data.pin_state[pin].value = analog_val;
                        ps = c->global_data.pin_state[pin];
                        pthread_mutex_unlock(&c->info_mutex);
                        process_callbacks(c, PIN_STATE_RESPONSE, &ps);
                    }
            }
            return;
        case DIGITAL_IO:
            if(msg.size >= 1)
            {
                /*
                 * Two byte digital data format, second nibble of byte 0 gives the port number (eg 0x92 is the third port, port 2)
                 * ```
                 * 0  digital data, 0x90-0x9F, (MIDI NoteOn, bud different data format)
                 * 1  digital pins 0-6 bitmask
                 * 2  digital pin 7 bitmask
                 * ```
                 */
                int port_num = (msg.start_code & 0x0F);
                int port_val = msg.msg_type | (msg.data[0] << 7);
                int pin = port_num * 8;
                int mask;
                for (mask=1; mask & 0xFF; mask <<= 1, pin++) {
                    pthread_mutex_lock(&c->info_mutex);
                    if (c->global_data.pin_state[pin].mode == MODE_INPUT) {
                        uint32_t val = (port_val & mask) ? 1 : 0;
                        if (c->global_data.pin_state[pin].value != val)
                        {
                            struct pin_state_msg ps;
                            c->global_data.pin_state[pin].value = val;
                            ps = c->global_data.pin_state[pin];
                            pthread_mutex_unlock(&c->info_mutex);
                            process_callbacks(c, PIN_STATE_RESPONSE, &ps);
                        }
                        else
                            pthread_mutex_unlock(&c->info_mutex);
                    }
                    else
                        pthread_mutex_unlock(&c->info_mutex);
                }
            }
            return;
        case REPORT_ANALOG_PIN:
        case REPORT_DIGITAL_PORT:
            fprintf(stderr, "Message %x is a command ?!?\n", cmd);
            return;
        default:
            if(VERSION_REPORT == msg.start_code)
            {
                /*
                 * Version report format
                 * ```
                 * 0  version report header (0xF9) (MIDI Undefined)
                 * 1  major version (0-127)
                 * 2  minor version (0-127)
                 * ```
                 */
                struct protocol_version_msg m;
                m.start_code = msg.start_code;
                m.major = msg.msg_type;
                m.minor = msg.data[0];
                pthread_mutex_lock(&c->info_mutex);
                c->global_data.protocol_version = m;
                pthread_mutex_unlock(&c->info_mutex);
                process_callbacks(c, VERSION_REPORT, &m);
                return;
            }
            switch(msg.msg_type)
            {
                case REPORT_FIRMWARE:
                    msg_to_firmware(&msg, &fw);
                    pthread_mutex_lock(&c->info_mutex);
                    c->global_data.fw = fw;
                    pthread_mutex_unlock(&c->info_mutex);
                    arg = &fw;

                    firmata_analog_mapping_query(c);
                    firmata_capability_query(c);
                    firmata_get_protocol_version(c);

                    break;
                case CAPABILITY_RESPONSE:
                    msg_to_pin_info(&msg, &pi);
                    pthread_mutex_lock(&c->info_mutex);
                    c->global_data.pin_cap = pi;
                    pthread_mutex_unlock(&c->info_mutex);
                    ask_pin_info(&pi, c);
                    arg = &pi;
                    if(!c->ready)
                    {
                        pthread_mutex_lock(&c->ready_mutex);
                        c->ready = 1;
                        pthread_cond_signal(&c->ready_cond);
                        pthread_mutex_unlock(&c->ready_mutex);
                    }
                    break;
                case ANALOG_MAPPING_RESPONSE:
                    msg_to_analog_map(&msg, &am);
                    pthread_mutex_lock(&c->info_mutex);
                    c->global_data.analog_map = am;
                    pthread_mutex_unlock(&c->info_mutex);
                    /* do not report analog channels by default */
                    for(i=0;i<MAX_NUM_PINS;i++)
                        if(am.analog_channel[i] != 127)
                            firmata_report_analog_channel(c, am.analog_channel[i], 0);
                    arg = &am;
                    break;
                case PIN_STATE_RESPONSE:
                    msg_to_pin_state(&msg, &ps);
                    pthread_mutex_lock(&c->info_mutex);
                    c->global_data.pin_state[ps.num] = ps;
                    pthread_mutex_unlock(&c->info_mutex);
                    arg = &ps;
                    break;
                case I2C_REPLY:
                    msg_to_i2c_msg(&msg, &i2c_msg);
                    arg = &i2c_msg;
                    break;
                case ENCODER_DATA:
                    pthread_mutex_lock(&c->info_mutex);
                    msg_to_enc_values(&msg, &c->global_data.encoders);
                    ev = c->global_data.encoders;
                    pthread_mutex_unlock(&c->info_mutex);
                    arg = &ev;
                    break;
                default:
                    break;
            }
            process_callbacks(c, msg.msg_type, arg);
    }
}

static void* waiter_thread(void * a)
{
    struct firmata_conn *c = a;
    struct pollfd pfd = { .fd = c->fd, .events = POLLIN | POLLERR };

    while(poll(&pfd, 1, -1) >= 0)
        process_message(c);
    return NULL;
}

struct firmata_conn *firmata_open(const char* devname, int baudrate)
{
    struct termios settings;
    struct serial_struct kernel_serial_settings;
    struct firmata_conn *c = NULL;
    struct timespec ts;
    int ret, bits, i, retry;

    if(!(c = malloc(sizeof(struct firmata_conn))))
    {
        perror("malloc");
        return NULL;
    }

    memset(c, 0, sizeof(*c));
    for(i=0; i < MAX_NUM_PINS; i++)
    {
        /* analog channel of 127 means invalid */
        c->global_data.analog_map.analog_channel[i] = 127;
        /* we use pin number 255 to mean not assigned */
        c->global_data.pin_state[i].num = 255;
    }

    pthread_mutex_init(&c->write_mutex, NULL);
    pthread_mutex_init(&c->cb_mutex, NULL);
    pthread_mutex_init(&c->info_mutex, NULL);

    if((c->fd = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    {
        perror("open");
        return NULL;
    }

    if((ret = ioctl(c->fd, TIOCMGET, &bits)) < 0)
    {
        perror("query serial port signals");
        goto open_fail;
    }

    bits &= ~(TIOCM_DTR | TIOCM_RTS);
    if((ret = ioctl(c->fd, TIOCMSET, &bits)) < 0)
    {
        perror("control serial port signals");
        goto open_fail;
    }

    if((ret = tcgetattr(c->fd, &c->orig_attribs)) < 0)
    {
        perror("tcgetattr");
        goto open_fail;
    }

    memset(&settings, 0, sizeof(settings));
    settings.c_iflag = IGNBRK | IGNPAR;
    settings.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;

    cfsetospeed(&settings, rate_to_constant(baudrate));
    cfsetispeed(&settings, rate_to_constant(baudrate));

    if(tcsetattr(c->fd, TCSANOW, &settings) < 0)
    {
        perror("tcsetattr");
        goto open_fail;
    }

    if(ioctl(c->fd, TIOCGSERIAL, &kernel_serial_settings) == 0)
    {
        kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
        ioctl(c->fd, TIOCSSERIAL, &kernel_serial_settings);
    }
    tcflush(c->fd, TCIFLUSH);

    pthread_mutex_init(&c->ready_mutex, NULL);
    pthread_cond_init (&c->ready_cond , NULL);
    pthread_mutex_lock(&c->ready_mutex);

    pthread_create(&c->thd, NULL, waiter_thread, c);

    ret = ETIMEDOUT;
    retry = 5;
    while(ret == ETIMEDOUT && retry > 0)
    {
        firmata_get_firmware_version(c);
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        ret = pthread_cond_timedwait(&c->ready_cond, &c->ready_mutex, &ts);
        if(ret == ETIMEDOUT && retry < 3)
        {
            fprintf(stderr, "Timedout while waiting for response. Retrying another %i time(s)\n", retry);
            retry--;
        }
    }
    if(ret) goto open_fail;

    return c;

open_fail:
    firmata_close(c);
    return NULL;
}

void firmata_close(struct firmata_conn *c)
{
    tcsetattr(c->fd, TCSANOW, &c->orig_attribs);
    close(c->fd);
    pthread_cancel(c->thd);
    pthread_join(c->thd, NULL);
    pthread_mutex_destroy(&c->write_mutex);
    pthread_mutex_destroy(&c->cb_mutex);
    pthread_mutex_destroy(&c->info_mutex);
    pthread_cond_destroy(&c->ready_cond);
    pthread_mutex_destroy(&c->ready_mutex);
    free(c);
}

int firmata_add_callback(struct firmata_conn *c, int code, void (*cb)(void* arg))
{
    int ret;
    pthread_mutex_lock(&c->cb_mutex);
    if(c->num_callbacks[code] >= MAX_CALLBACKS - 1)
        ret = ENOMEM;
    else
    {
        c->callbacks[code][c->num_callbacks[code]++] = cb;
        ret = 0;
    }
    pthread_mutex_unlock(&c->cb_mutex);
    return ret;
}

void firmata_del_callback(struct firmata_conn *c, int code, void (*cb)(void* arg))
{
    int i;
    pthread_mutex_lock(&c->cb_mutex);
    for(i=0; i < c->num_callbacks[code]; i++)
    {
        if(c->callbacks[code][i] == cb)
        {
            if(c->num_callbacks[code] > 1)
            {
                c->callbacks[code][i] = c->callbacks[code][c->num_callbacks[code]-1];
                c->num_callbacks[code]--;
                c->callbacks[code][c->num_callbacks[code]] = NULL;
            }
            else
                c->callbacks[code][0] = NULL;
        }
    }
    pthread_mutex_unlock(&c->cb_mutex);
}
