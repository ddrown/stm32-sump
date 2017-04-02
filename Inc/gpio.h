#ifndef GPIO_H
#define GPIO_H

#define GPIO_BUFFER_SIZE 10240
extern uint8_t gpio_buffer[];
void do_gpio_dma();

#endif
