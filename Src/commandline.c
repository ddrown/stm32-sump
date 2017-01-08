#include "stm32f1xx_hal.h"
#include "commandline.h"
#include "syscall.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdlib.h>

static void command_memory() {
  printf("heap = %lu/%lu stacksize = %lu/%lu\n", heapsize(), heapmax(), stacksize(), stackmax());
}

static void command_serial() {
  printf("output timeouts = %lu input overruns = %lu\n", timeouts, overruns);
}

static void blink(const char *option) {
  if(strcmp(option, " on") == 0) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  } else if(strcmp(option, " off") == 0) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

static void print_help() {
  printf("commands:\n");
  printf("blink (on|off|toggle) - blink LED\n");
  printf("mem - show mem\n");
  printf("serial - show usb serial stats\n");
}

static void run_command(char *cmdline) {
  if(strncmp("blink", cmdline,5) == 0) {
    blink(cmdline+5);
  } else if(strcmp("mem", cmdline) == 0) {
    command_memory();
  } else if(strcmp("serial", cmdline) == 0) {
    command_serial();
  } else {
    print_help();
  }
}

void cmdline_prompt() {
  _write(0, "> ", 2);
}

static char cmdline[40];
static uint32_t cmd_len = 0;

void reprint_prompt() {
  _write(0, "> ", 2);
  if(cmd_len > 0) {
    _write(0, cmdline, cmd_len);
  }
}

void cmdline_addchr(char c) {
  switch(c) {
    case '\r':
    case '\n':
      _write(0, "\n", 1);
      run_command(cmdline);
      cmdline_prompt();
      cmdline[0] = '\0';
      cmd_len = 0;
      break;
    case '\b':
    case '\x7F':
      if(cmd_len > 0) {
        _write(0, "\x7F", 1);
        cmd_len--;
        cmdline[cmd_len] = '\0';
      }
      break;
    default:
      if(cmd_len < sizeof(cmdline)-1) {
        _write(0, &c, 1);
        cmdline[cmd_len] = c;
        cmdline[cmd_len+1] = '\0';
        cmd_len = cmd_len + 1;
      }
  }
}
