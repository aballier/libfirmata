# Copyright (c) 2015, Alexis Ballier.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the <ORGANIZATION> nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -I $(top_srcdir)/include

lib_LTLIBRARIES = libfirmata.la

libfirmata_la_SOURCES = \
	src/descriptions.c \
	src/libfirmata.c \
	src/output.c

libfirmata_la_LDFLAGS = -version-info 0:0:0
libfirmata_la_CFLAGS  = -pthread
noinst_HEADERS = src/libfirmata_internal.h

bin_PROGRAMS = firmata_get_pincaps
firmata_get_pincaps_SOURCES = apps/pincaps.c
firmata_get_pincaps_CFLAGS = -pthread
firmata_get_pincaps_LDADD  = libfirmata.la

firmatainclude_HEADERS = \
	include/firmata/libfirmata.h \
	include/firmata/protocol.h

firmataincludedir = ${prefix}/include/firmata

htmldoc: $(top_builddir)/Doxyfile
	doxygen $<
