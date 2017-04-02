#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "syscall.h"
#include <string.h>
#include <errno.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint32_t SystemCoreClock;

uint32_t timeouts = 0;

int _write(int fd, const void *buf, size_t count) {
  uint32_t cycles = 0;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
  while(hcdc->TxState == 1) {
    if(cycles++ > SystemCoreClock/130) { // ~100ms
      timeouts++;
      return count; // don't retry
    }
  }
  CDC_Transmit_FS((uint8_t *)buf, count);

  return count;
}

/* memory functions
 *******************************************
 */
static char heap_start[1024];

/*  sbrk - Increase program data space.
 *  newlib's malloc and related functions depend on this (like printf)
 */
caddr_t _sbrk(int incr) {
  static char *heap_end;
  char *prev_heap_end;

  if (heap_end == 0) {
    heap_end = heap_start;
  }
  if(incr == 0) {
    return (caddr_t) heap_end;
  }
  prev_heap_end = heap_end;

  if ((heap_end + incr) >= (heap_start + sizeof(heap_start))) {
    errno = ENOMEM;
    return  (caddr_t) -1;
    //abort ();
  }

  heap_end += incr;
  return (caddr_t) prev_heap_end;
}

uint32_t heapsize() {
  return _sbrk(0) - heap_start;
}

uint32_t heapmax() {
  return sizeof(heap_start);
}

// value for stm32f103c8 (20KB)
#define STACK_BOTTOM 0x20005000

uint32_t stacksize() {
  char *stack_top = (char *)__get_MSP();
  char *stack_bottom = (char *)STACK_BOTTOM;
  return stack_bottom - stack_top;
}

uint32_t stackmax() {
  extern char _ebss; // top of heap from linker
  char *stack_bottom = (char *)STACK_BOTTOM;
  return stack_bottom - &_ebss;
}
