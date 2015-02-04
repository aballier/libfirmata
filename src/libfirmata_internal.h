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

#ifndef __LIBFIRMATA_INTERNAL_H
#define __LIBFIRMATA_INTERNAL_H

#include <termios.h>
#include <pthread.h>

#include "firmata/libfirmata.h"

#define MAX_CALLBACKS 20

/**
 * @brief Represents the state of a connection to a firmata device.
 */
struct firmata_conn {
    /*! File descriptor of the serial connection */
    int fd;
    /*! Original termios attribs */
    struct termios orig_attribs;

    /*! Reader thread */
    pthread_t thd;

    /*! Mutex for writing to the serial port */
    pthread_mutex_t write_mutex;

    /*! Mutex for accessing callback structures */
    pthread_mutex_t cb_mutex;

    /*! Callbacks */
    void (*callbacks[255][MAX_CALLBACKS])(void* arg);

    /*! Number of callbacks per type */
    size_t num_callbacks[255];

    /*! Is the connection ready ? */
    int ready;
    /*! Mutex to lock when waiting on ready_cond */
    pthread_mutex_t ready_mutex;
    /*! Condition to wait for ready state */
    pthread_cond_t  ready_cond;

    /*! Mutex to lock when accessing below info */
    pthread_mutex_t info_mutex;

    /*! Information fetched asynchronously */
    struct firmata_global_data global_data;
};

void process_callbacks(struct firmata_conn *c, uint8_t code, void *arg);

#endif /* __LIBFIRMATA_INTERNAL_H */
