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

#include "firmata/libfirmata.h"
#include "libfirmata_internal.h"

const char* firmata_get_msg_type_desc(struct firmata_msg* msg)
{
	static const char const * cmd_desc[256] = {
		[START_SYSEX]         = "Start a MIDI Sysex message",
		[SET_PIN_MODE]        = "Set pin mode",
		[END_SYSEX]           = "End a MIDI Sysex message",
		[VERSION_REPORT]      = "Protocol version report",
		[ANALOG_IO]           = "Analog I/O message",
		[DIGITAL_IO]          = "Digital I/O message",
		[REPORT_ANALOG_PIN]   = "Report analog pin",
		[REPORT_DIGITAL_PORT] = "Report digital port",
		NULL
	};

	if(cmd_desc[msg->start_code & 0xF0])
		return cmd_desc[msg->start_code & 0xF0];
	return cmd_desc[msg->start_code];
}

const char* firmata_get_pin_mode_desc(uint8_t mode)
{
	static const char const * mode_desc[MODE_MAX] = {
		[MODE_INPUT  ] = "Input",
		[MODE_OUTPUT ] = "Output",
		[MODE_ANALOG ] = "Analogic",
		[MODE_PWM    ] = "Pulse width modulation",
		[MODE_SERVO  ] = "Servo",
		[MODE_SHIFT  ] = "Shift",
		[MODE_I2C    ] = "I2C",
		[MODE_ONEWIRE] = "One wire",
		[MODE_STEPPER] = "Stepper",
		[MODE_ENCODER] = "Encoder",
	};

	return (mode < MODE_MAX ? mode_desc[mode] : NULL );
}

ssize_t firmata_fill_supported_modes(uint64_t supported_modes, const char** values, ssize_t len)
{
	ssize_t pos = 0;
	unsigned int i;
	for(i=0;i<MODE_MAX && pos < len;i++)
		if(supported_modes & (1 << i))
			values[pos++] = firmata_get_pin_mode_desc(i);
	return pos;
}
