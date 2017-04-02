#include "stm32f1xx_hal.h"
#include "gpio.h"

extern TIM_HandleTypeDef htim1;

uint8_t gpio_buffer[GPIO_BUFFER_SIZE];

void do_gpio_dma() {
  HAL_StatusTypeDef state;

  HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&GPIOA->IDR, (uint32_t)&gpio_buffer, sizeof(gpio_buffer));
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  state = HAL_DMA_PollForTransfer(htim1.hdma[TIM_DMA_ID_UPDATE], HAL_DMA_FULL_TRANSFER, 10);
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if(state != HAL_OK) {
    printf("DMA state %u\n", state);
  }
}
