#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "uart.h"

uint8_t gpio_buffer[GPIO_BUFFER_SIZE];

static inline void wait_for_trigger(uint8_t trigger_mask, uint8_t trigger_values) {
  if(trigger_mask) {
    while((GPIOA->IDR ^ trigger_values) & trigger_mask); // wait until trigger
  }
}

void do_gpio_dma(uint32_t length, uint8_t trigger_mask, uint8_t trigger_values) {
  HAL_StatusTypeDef state;

  if(length > sizeof(gpio_buffer)) {
    length = sizeof(gpio_buffer);
  }

  wait_for_trigger(trigger_mask, trigger_values); // TODO: switch this to watching the DMA values

  HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&GPIOA->IDR, (uint32_t)&gpio_buffer, length);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  HAL_DMA_PollForTransfer(htim1.hdma[TIM_DMA_ID_UPDATE], HAL_DMA_FULL_TRANSFER, 100);
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
}

void do_gpio_loop(uint32_t length, uint8_t trigger_mask, uint8_t trigger_values) {
  uint8_t *buffer = gpio_buffer;
  uint8_t *ending;

  if(length > sizeof(gpio_buffer)) {
    length = sizeof(gpio_buffer);
  }
  ending = gpio_buffer+length;

  wait_for_trigger(trigger_mask, trigger_values);

  __disable_irq();

// good for 7.2MHz
// check Src/read256.c for a higher speed version (~12MHz) that uses more flash
    asm(
      "ldr     r1, [%1]\n"
      "poll:"
      "strb    r1, [%0, #0]\n"
      "adds    %0, #1\n"
      "cmp     %2, %0\n"
      "ldr     r1, [%1]\n" // moving this before the cmp slows the loop down to 6MHz
      "bhi.n   poll\n"
    : /* no outputs */
    : "r" (buffer), "r" (&GPIOA->IDR), "r" (ending)
    : "r1"
    );

  __enable_irq();
}
