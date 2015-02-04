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
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>

#include <firmata/libfirmata.h>

sem_t sem;

static void pin_info_cb(void *arg)
{
    const char* modes[MODE_MAX];
    struct pin_cap_msg *pi = arg;
    int i;
    for(i=0;i<MAX_NUM_PINS; i++)
    {
        int len;
        if( (len = firmata_fill_supported_modes(pi->pin_info[i].supported_modes, modes, MODE_MAX)) > 0)
        {
            int j;
            printf("Supported modes for pin %i: ", i);
            for(j=0;j<len;j++)
                printf("'%s', ", modes[j]);
            printf("\n");
        }
    }
    sem_post(&sem);
}

int main(int argc, char *argv[]) {
    struct firmata_conn *c = NULL;
    const char* devname;
    int baudrate;
    sem_init(&sem, 0, 0);

    if(argc > 1)
        devname = argv[1];
    else
        devname = "/dev/ttySAC0";

    if(argc > 2)
        baudrate = atoi(argv[2]);
    else
        baudrate = 57600;

    if(!(c = firmata_open(devname, baudrate)))
    {
        fprintf(stderr, "Failed to open the device %s at rate %i\n", devname, baudrate);
        return EINVAL;
    }

    firmata_add_callback(c, CAPABILITY_RESPONSE, pin_info_cb);
    firmata_capability_query(c);

    sem_wait(&sem);
    firmata_close(c);
    sem_destroy(&sem);
    return 0;
}
