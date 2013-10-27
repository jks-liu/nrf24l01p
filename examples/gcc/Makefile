#******************************************************************************
#
# Makefile - Rules for building the project example.
#
# Copyright (c) 2013 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the  
#   distribution.
# 
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# This is part of revision 1.0 of the Tiva Firmware Development Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=TM4C1233H6PM

VPATH += ../../utils
VPATH += lisp-style-repl

#
# The base directory for TivaWare.
#
ROOT=../..

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH=../..

#
# The default rule, which causes the project example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/main.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

CFLAGS += -DUART_BUFFERED
#
# Rules for building the project example.
#
${COMPILER}/main.axf: ${COMPILER}/main.o
${COMPILER}/main.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/main.axf: ${COMPILER}/delay.o
${COMPILER}/main.axf: ${COMPILER}/nrf24l01+.o
${COMPILER}/main.axf: ${COMPILER}/uartstdio.o
${COMPILER}/main.axf: ${COMPILER}/lisp-style-repl.o
${COMPILER}/main.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/main.axf: main.ld
SCATTERgcc_main=main.ld
ENTRY_main=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RA3

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
