# Copyright (c) 2024 Ian cohee
#
# BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# @file    : Makefile
# @author  : Ian Cohee
# @date    : 2024-30-09
# @version : v0.1.0
# @brief   : Source Makefile
#

AR=ar
AR_FLAGS=rcs

CC=gcc
CC_FLAGS=-Wall -Wextra -pedantic

LD_SO_FLAGS=-shared -fPIC 

RM=rm

SGP30_O=sgp30.o
LIBSGP30_A=libSgp30.a
LIBSGP30_SO=libSgp30.so

SGP30_LIBS=$(LIBSGP30_A) $(LIBSGP30_SO)

all: $(SGP30_LIBS)

$(SGP30_O): sgp30.c
	$(CC) $(CC_FLAGS) -c $<

libSgp30.so: $(SGP30_O)
	$(CC) $(LD_SO_FLAGS) -o $@ $<

libSgp30.a: $(SGP30_O)
	$(AR) $(AR_FLAGS) $@ $<

clean:
	$(RM) -v $(SGP30_O) $(SGP30_LIBS)