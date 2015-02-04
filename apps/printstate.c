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
#include <errno.h>
#include <unistd.h>

#include <firmata/libfirmata.h>

int main(int argc, char *argv[]) {
    struct firmata_conn *c = NULL;
    const struct firmata_global_data *s = NULL;
    const char* devname;
    int baudrate;

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

    /* ask for encoders to report their positions */
    firmata_encoder_report_positions(c);
 
    /* wait so that events can be received */
    sleep(1);

    s = firmata_get_global_state(c);

    printf("Protocol version = %i.%i\n", s->protocol_version.major, s->protocol_version.minor);
    printf("Firmware name = '%s', version = %i.%i\n", s->fw.name, s->fw.major, s->fw.minor);

    {
        const char* modes[MODE_MAX];
        int i;
        printf("Pin capabilities:\n");
        for(i=0;i<MAX_NUM_PINS; i++)
        {
            int len;
            if( (len = firmata_fill_supported_modes(s->pin_cap.pin_info[i].supported_modes, modes, MODE_MAX)) > 0)
            {
                int j;
                printf("\tSupported modes for pin %i: ", i);
                for(j=0;j<len;j++)
                    printf("'%s', ", modes[j]);
                printf("\n");
            }
        }
    }

    {
        int i;
        printf("Analog mapping:\n");
        for(i=0;i<MAX_NUM_PINS;i++)
            if(s->analog_map.analog_channel[i] < 127 && s->pin_state[i].num < 255)
                printf("\tPin %i is mapped to analog channel %i\n", i, s->analog_map.analog_channel[i]);
    }

    {
        int i;
        printf("Pin states:\n");
        for(i=0;i<MAX_NUM_PINS;i++)
            if(s->pin_state[i].num < 255)
                printf("\tPin %i is in mode %s and has value %u\n", i, firmata_get_pin_mode_desc(s->pin_state[i].mode), s->pin_state[i].value);
    }

    {
        int i;
        printf("Encoders:\n");
        for(i=0;i<MAX_ENCODERS;i++)
            printf("\tEncoder %i is at position %u with direction %u\n", i, s->encoders.encoders[i].position, s->encoders.encoders[i].direction);
    }

    if(firmata_put_global_state(c, s))
    {
        fprintf(stderr, "firmata_put_global_state failed\n");
    }
    firmata_close(c);
    return 0;
}
