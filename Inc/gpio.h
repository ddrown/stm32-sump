#ifndef GPIO_H
#define GPIO_H

#define GPIO_BUFFER_SIZE 10240
extern uint8_t gpio_buffer[];
void do_gpio_dma(uint32_t length, uint8_t trigger_mask, uint8_t trigger_values);
void do_gpio_loop(uint32_t length, uint8_t trigger_mask, uint8_t trigger_values);

extern TIM_HandleTypeDef htim1;

#endif
