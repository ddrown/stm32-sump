#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "uart.h"

uint8_t gpio_buffer[GPIO_BUFFER_SIZE];

void do_gpio_dma(uint32_t length) {
  HAL_StatusTypeDef state;

  if(length > sizeof(gpio_buffer)) {
    length = sizeof(gpio_buffer);
  }

  HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&GPIOA->IDR, (uint32_t)&gpio_buffer, length);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  state = HAL_DMA_PollForTransfer(htim1.hdma[TIM_DMA_ID_UPDATE], HAL_DMA_FULL_TRANSFER, 110);
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if(state != HAL_OK) {
    write_uart_s("DMA state ");
    write_uart_u(state);
    write_uart_s("\n");
  }
}

void do_gpio_loop(uint32_t length) {
  uint8_t *buffer = gpio_buffer;
  if(length > sizeof(gpio_buffer)) {
    length = sizeof(gpio_buffer);
  }

  __disable_irq();

  do {
  // read 5KB at a time, jump takes too many cycles
  // 5KB at a time = 50KB flash used
  // 2KB at a time = 32KB flash used
  // 2KB version will slip ~83ns every ~170us or ~486ppm off due to jump
  // 2KB version results in a 100KHz synchronus clock from TIM3 showing up as 100.037KHz
  // alternative to test: use code in memory?
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
#include "read256.c"
  } while(buffer < gpio_buffer+length);

  __enable_irq();
}
