/*
 *
 * SUMP Protocol Implementation for Arduino boards.
 *
 * Copyright (c) 2011,2012,2013,2014,2015 Andrew Gillham
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ANDREW GILLHAM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANDREW GILLHAM BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

// this code is based on:
// https://github.com/gillham/logic_analyzer/blob/master/logic_analyzer.ino
// modified to work under stm32 HAL

#include "stm32f1xx_hal.h"
#include "sump.h"
#include "gpio.h"

#include <string.h>
#include <unistd.h>

#define SUMP_RESET 0x00
#define SUMP_ARM   0x01
#define SUMP_QUERY 0x02
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK 0xC0
#define SUMP_TRIGGER_VALUES 0xC1
#define SUMP_TRIGGER_CONFIG 0xC2

/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

static uint8_t cmdBytes[3];

static void getCmd() {
  // TODO
}

static void setupDelay() {
  // TODO
}

static void get_metadata() {
  /* device name */
  printf("%cAGLASv0%c", 1, 0);
  /* firmware version */
  printf("%c0.13%c", 2, 0);
  /* sample memory */
  printf("%c%c%c", 0x21, 0, 0);
  /* 7168 bytes */
  printf("%c%c", 0x1C, 0);

  /* sample rate (4MHz) */
  printf("%c%c%c%c%c", 0x23, 0, 0x3D, 0x9, 0);

  /* number of probes (6 by default on Arduino, 8 on Mega) */
  printf("%c%c",0x40, 0x08);

  /* protocol version (2) */
  printf("%c%c", 0x41, 0x2);

  /* end of data */
  printf("%c", 0x00);
}

void sump_cmd(char c) {
  uint32_t trigger, trigger_values, divider, readCount, rleEnabled, delayCount;

  switch (c) {
    case SUMP_RESET:
      /*
       * We don't do anything here as some unsupported extended commands have
       * zero bytes and are mistaken as resets.  This can trigger false resets
       * so we don't erase the data or do anything for a reset.
       */
      break;
    case SUMP_QUERY:
      /* return the expected bytes. */
      printf("1ALS");
      break;
    case SUMP_ARM:
      /*
       * Zero out any previous samples before arming.
       * Done here instead via reset due to spurious resets.
       */
      memset(gpio_buffer, '\0', GPIO_BUFFER_SIZE);
      do_gpio_dma(); // TODO: triggers
      write(0, gpio_buffer, GPIO_BUFFER_SIZE); // TODO: does this work? does it need to be cut into 64 byte pieces?
      break;
    case SUMP_TRIGGER_MASK:
      /*
       * the trigger mask byte has a '1' for each enabled trigger so
       * we can just use it directly as our trigger mask.
       */
      getCmd();
      trigger = cmdBytes[0];
      break;
    case SUMP_TRIGGER_VALUES:
      /*
       * trigger_values can be used directly as the value of each bit
       * defines whether we're looking for it to be high or low.
       */
      getCmd();
      trigger_values = cmdBytes[0];
      break;
    case SUMP_TRIGGER_CONFIG:
      /* read the rest of the command bytes, but ignore them. */
      getCmd();
      break;
    case SUMP_SET_DIVIDER:
      /*
       * the shifting needs to be done on the 32bit unsigned long variable
       * so that << 16 doesn't end up as zero.
       */
      getCmd();
      divider = cmdBytes[2];
      divider = divider << 8;
      divider += cmdBytes[1];
      divider = divider << 8;
      divider += cmdBytes[0];
      setupDelay(); // TODO
      break;
    case SUMP_SET_READ_DELAY_COUNT:
      /*
       * this just sets up how many samples there should be before
       * and after the trigger fires.  The readCount is total samples
       * to return and delayCount number of samples after the trigger.
       * this sets the buffer splits like 0/100, 25/75, 50/50
       * for example if readCount == delayCount then we should
       * return all samples starting from the trigger point.
       * if delayCount < readCount we return (readCount - delayCount) of
       * samples from before the trigger fired.
       */
      getCmd();
      readCount = 4 * (((cmdBytes[1] << 8) | cmdBytes[0]) + 1);
      if (readCount > GPIO_BUFFER_SIZE)
        readCount = GPIO_BUFFER_SIZE;
      delayCount = 4 * (((cmdBytes[3] << 8) | cmdBytes[2]) + 1);
      if (delayCount > GPIO_BUFFER_SIZE)
        delayCount = GPIO_BUFFER_SIZE;
      break; // TODO
    case SUMP_SET_FLAGS:
      /* read the rest of the command bytes and check if RLE is enabled. */
      getCmd();
      rleEnabled = ((cmdBytes[1] & 0b1000000) != 0);
      break; // TODO
    case SUMP_GET_METADATA:
      /*
       * We return a description of our capabilities.
       * Check the function's comments below.
       */
      get_metadata();
      break;
    case SUMP_SELF_TEST:
      /* ignored. */
      break;
    default:
      /* ignore any unrecognized bytes. */
      break;
  }
}
