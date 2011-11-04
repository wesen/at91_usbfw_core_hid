#
# Port for GNU-make by Martin Thomas (not Atmel)
# This version will not compile on a Unix/Linux-System
# since the Windows-"Shell" (cmd.exe) is used by some targets - TODO
# -> still a lot of TODOs but most of it works

#/* ----------------------------------------------------------------------------
# *         ATMEL Microcontroller Software Support  -  ROUSSET  -
# * ----------------------------------------------------------------------------
# * Copyright (c) 2006, Atmel Corporation
#
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * - Redistributions of source code must retain the above copyright notice,
# * this list of conditions and the disclaiimer below.
# *
# * - Redistributions in binary form must reproduce the above copyright notice,
# * this list of conditions and the disclaimer below in the documentation and/or
# * other materials provided with the distribution.
# *
# * Atmel`s name may not be used to endorse or promote products derived from
# * this software without specific prior written permission.
# *
# * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# * ----------------------------------------------------------------------------
# *
#
# $Id: Makefile 135 2006-10-19 15:50:25Z jjoannic $
#
#-------------------------------------------------------------------------------
#       Parameters
#-------------------------------------------------------------------------------
#
#   TARGET:     target chip         (default: TARGET=AT91SAM7S64)
#   BOARD:      board used          (default: BOARD=AT91SAM7SEK)
#   CLASS:      USB class           (default: CLASS=ENUM)
#   MODE:       Media or subclass   (default: MODE=NO)
#   REMAP:      Run from SRAM       (default: REMAP=NO)
#   DEBUG:      Debug symbols       (default: DEBUG=NO)
#   LEDS:       Use leds            (default: LEDS=YES)
#   TRACES:     Output traces       (default: TRACES=YES)
#   POWER:      Self/bus powered    (default: POWER=AUTO)
#   REVISION:   Board revision      (default: REV_B)
#-------------------------------------------------------------------------------

# SHELL = cmd.exe

#MAKEDIR = .
include mt_defs.mk

VALIDTARGETS_ARM7 = \
  AT91SAM7S321 AT91SAM7S64   AT91SAM7S128 AT91SAM7S256 AT91SAM7S512 \
  AT91SAM7SE32 AT91SAM7SE256 AT91SAM7SE512 \
  AT91SAM7X128 AT91SAM7X256  AT91SAM7X512  \
  AT91SAM7A3
VALIDTARGETS_ARM9 =  AT91RM9200   AT91SAM9260  AT91SAM9261   AT91SAM9263
VALIDTARGETS = $(VALIDTARGETS_ARM7) $(VALIDTARGETS_ARM9)

#-------------------------------------------------------------------------------
#       Check parameters
#-------------------------------------------------------------------------------

TARGET = AT91SAM7S256

#-------------------------------------------------------------------------------
#       Check target
#-------------------------------------------------------------------------------
ifndef TARGET
$(warning Warning: No target selected, using default target AT91SAM7S64.)
TARGET = AT91SAM7S64
endif

ifeq (,$(filter $(TARGET), $(VALIDTARGETS)))
$(error Error: $(TARGET): Unknown target.)
endif

ifeq (,$(filter $(TARGET), $(VALIDTARGETS_ARM9)))
MCPU = arm7tdmi
else
#todo: check this (must read - no AT91 ARM9 here for tests)
MCPU = arm9
endif


#-------------------------------------------------------------------------------
#       Check board
#-------------------------------------------------------------------------------
ifndef BOARD
$(warning Warning: No board selected, using default board AT91SAM7SEK.)
BOARD = AT91SAM7SEK
endif

#-------------------------------------------------------------------------------
#       Check class
#-------------------------------------------------------------------------------
ifndef CLASS
$(warning Warning: No class selected, using default enumeration class.)
CLASS = ENUM
endif

#-------------------------------------------------------------------------------
#       Check remap
#-------------------------------------------------------------------------------
ifndef REMAP
REMAP = NO
else
ifneq ($(REMAP),YES)
REMAP = NO
endif
endif

#-------------------------------------------------------------------------------
#       Check debug
#-------------------------------------------------------------------------------
ifndef DEBUG
DEBUG = NO
else
ifneq ($(DEBUG),YES)
DEBUG = NO
endif
endif

#-------------------------------------------------------------------------------
#       Check leds
#-------------------------------------------------------------------------------
ifndef LEDS
LEDS = YES
else
ifneq ($(LEDS),NO)
LEDS = YES
endif
endif

#-------------------------------------------------------------------------------
#       Check traces
#-------------------------------------------------------------------------------
ifndef TRACES
TRACES = YES
else
ifneq ($(TRACES),NO)
TRACES = YES
endif
endif

#-------------------------------------------------------------------------------
#       Check power
#-------------------------------------------------------------------------------
ifndef POWER
POWER = AUTO
else
ifneq ($(POWER),BUS)
POWER = AUTO
endif
endif

#-------------------------------------------------------------------------------
#       Check mode
#-------------------------------------------------------------------------------
ifndef MODE
MODE = NO
endif

#-------------------------------------------------------------------------------
#       Summary
#-------------------------------------------------------------------------------

$(warning Target:$(TARGET)/$(MCPU) Board:$(BOARD))
$(warning Class:$(CLASS) Mode:$(MODE) Power:$(POWER))
$(warning Remap:$(REMAP))
$(warning Debug:$(DEBUG) LEDs:$(LEDS) Traces:$(TRACES))

ifeq ($(TARGET),AT91SAM7A3)
ifndef REVISION
$(warning Revision: REV_B)
else
$(warning Revision: $(REVISION))
endif
endif

#-------------------------------------------------------------------------------
#       Memory
#-------------------------------------------------------------------------------

# skipped since just information - see original Makefile

#-------------------------------------------------------------------------------
#       Tools
#-------------------------------------------------------------------------------

TOOLCHAIN_PREFIX=arm-none-eabi
AS = $(TOOLCHAIN_PREFIX)-as
LD = $(TOOLCHAIN_PREFIX)-ld
FE = $(TOOLCHAIN_PREFIX)-objcopy
SIZE = $(TOOLCHAIN_PREFIX)-size
THUMB_CC = $(TOOLCHAIN_PREFIX)-gcc -mthumb
ARM_CC = $(TOOLCHAIN_PREFIX)-gcc
CC = $(THUMB_CC)

OBJDUMP = $(TOOLCHAIN_PREFIX)-objdump

#-------------------------------------------------------------------------------
#       Output directories
#-------------------------------------------------------------------------------

BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#       Files
#-------------------------------------------------------------------------------

STARTUP = startup
SYSCALLS = syscalls_gnu

#-------------------------------------------------------------------------------
#       Target-dependant parameters
#-------------------------------------------------------------------------------

# Start of memory regions
RAM_START = 0x00200000
ROM_START = 0x00100000

# Include directories
LIB = lib/$(TARGET)

#-------------------------------------------------------------------------------
#       Debug mode
#-------------------------------------------------------------------------------

ifeq ($(DEBUG),YES)
OPTIMIZATION = -O0 -gdwarf-2 -fno-inline
else
OPTIMIZATION = -O2
endif

#-------------------------------------------------------------------------------
#       Compilation flags
#-------------------------------------------------------------------------------

##todo
#ASFLAGS = -g -keep -PD "$(TARGET) SETA 1" -apcs /interwork -fpu None\
#          -i$(LIB)
# TODO arm9
ASFLAGS  = -mcpu=$(MCPU) -mthumb-interwork -I $(LIB) \
   -x assembler-with-cpp -D$(TARGET)
ifeq ($(DEBUG),YES)
ASFLAGS += -gdwarf-2
endif

ifeq ($(REMAP),YES)
##todo
#ASFLAGS = $(ASFLAGS) -PD "REMAP SETA 1"
endif

ifeq ($(DEBUG),YES)
##todo
#ASFLAGS = $(ASFLAGS) -PD "DEBUG SETA 1"
endif


#CCFLAGS = -apcs /interwork -gtp -Wb+avnglp -D__APCS_INTERWORK\
#          $(OPTIMIZATION) -D$(TARGET) -D$(BOARD) -I$(LIB) -I./

CCFLAGS = -mcpu=$(MCPU) -std=gnu99 -Wall -mthumb-interwork \
          $(OPTIMIZATION) -D$(TARGET) -D$(BOARD) -I./ \
          -ffunction-sections -fdata-sections

ifeq ($(LEDS),NO)
CCFLAGS := $(CCFLAGS) -DNOLEDS
endif

ifeq ($(TRACES),NO)
CCFLAGS := $(CCFLAGS) -DNOTRACES
endif

ifeq ($(POWER),BUS)
CCFLAGS := $(CCFLAGS) -DUSB_BUS_POWERED
endif

ifneq ($(MODE),NO)
CCFLAGS := $(CCFLAGS) -D$(MODE)
endif

#-------------------------------------------------------------------------------
#       Linker flags
#-------------------------------------------------------------------------------

##todo
#LDFLAGS = -entry __ENTRY -ro-base 0 -nolocals -first $(STARTUP).o(reset)\
#          -info totals

LDFLAGS =  -mthumb-interwork -nostartfiles 
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref,--gc-sections,--entry=__ENTRY

# TODO -> debug = "RAM-RUN" / non-debug = "ROM_RUN" 
# "mimic" ro-base / rw-base with parameters to ld
# fixed to ROM-RUN for now
LDFLAGS += -T../at91-ROM.ld

ifeq ($(REMAP),NO)
ifeq ($(DEBUG),NO)
## todo
#LDFLAGS = $(LDFLAGS) -rw-base $(RAM_START)
endif
endif

ifeq ($(DEBUG),NO)
# LDFLAGS := $(LDFLAGS) -nodebug
endif

#-------------------------------------------------------------------------------
#       Objects
#-------------------------------------------------------------------------------

COREOBJ = core.o

ifneq ($(CLASS),ENUM)
CLASSOBJ = $(CLASS).o
endif

#-------------------------------------------------------------------------------
#       Directives
#-------------------------------------------------------------------------------

# Binary
BINARY = $(BOARD)_$(CLASS)

ifneq ($(MODE),NO)
BINARY := $(BINARY)_$(MODE)
endif

ifeq ($(DEBUG),YES)
BINARY := $(BINARY)_DEBUG
endif

all: $(BIN)/$(TARGET)/$(BINARY)

$(BIN)/$(TARGET)/$(BINARY): $(STARTUP).o $(SYSCALLS).o $(COREOBJ) $(CLASSOBJ)
	@echo Building $@ ...
#	@if not exist $(BIN) mkdir $(BIN)
#	@if not exist $(BIN)\$(TARGET) mkdir $(BIN)\$(TARGET)
	cd $(OBJ) && $(CC) $(LDFLAGS) $(STARTUP).o $(SYSCALLS).o $(COREOBJ) $(CLASSOBJ) -o ../$(BINARY).elf
	$(OBJDUMP) -h -S -C $(BINARY).elf > $(BINARY).lss
	$(FE) -O binary $(BINARY).elf $(BIN)/$(TARGET)/$(BINARY).bin
	$(SIZE) -A $(BINARY).elf
ifeq ($(DEBUG),NO)
##	@del $(BINARY).elf
endif

$(STARTUP).o: $(STARTUP).s
	@echo Building $@ ...
	$(CC) -c $(ASFLAGS) $< -o $(OBJ)/$@

$(SYSCALLS).o: $(SYSCALLS).c
	@echo Building $@ ...
	$(CC) -c $(CCFLAGS) -o $(OBJ)/$@ $<

$(COREOBJ):
	cd core && $(MAKE) -f Makefile_GNU BOARD=$(BOARD) CLASS=$(CLASS) LIB=../$(LIB) \
	MODE=$(MODE) REMAP=$(REMAP) DEBUG=$(DEBUG) LEDS=$(LEDS) TRACES=$(TRACES) \
	POWER=$(POWER) OBJ=../$(OBJ) BIN=../$(BIN) TARGET=$(TARGET) \
	MCPU=$(MCPU)

ifneq ($(CLASS),ENUM)
$(CLASSOBJ):
	cd $(CLASS) && $(MAKE) -f Makefile_GNU BOARD=$(BOARD) CLASS=$(CLASS) LIB=../$(LIB) \
	MODE=$(MODE) REMAP=$(REMAP) DEBUG=$(DEBUG) LEDS=$(LEDS) TRACES=$(TRACES) \
	POWER=$(POWER) OBJ=../$(OBJ) BIN=../$(BIN) TARGET=$(TARGET) \
	MCPU=$(MCPU)
endif

rebuild: clean all

## program: $(BIN)/$(TARGET)/$(BINARY)
program:
	@copy $(BIN)\$(TARGET)\$(BINARY).bin tmp_image.bin
	openocd_go_flash.cmd 
	@del tmp_image.bin

clean:
	@echo Cleaning project ...
	@if exist $(OBJ)\*.o del $(OBJ)\*.o
	@if exist $(OBJ)\*.map del $(OBJ)\*.map
	@if exist $(BIN)\$(TARGET)\$(BINARY).* del $(BIN)\$(TARGET)\$(BINARY).*
	@if exist $(BINARY).elf del $(BINARY).elf
	@if exist $(BINARY).lss del $(BINARY).lss
