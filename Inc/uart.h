#ifndef UART_H
#define UART_H

extern UART_HandleTypeDef huart1;
#define UART_NAME huart1

void write_uart_s(const char *s);
void write_uart_u(uint32_t i);
void write_uart_i(int32_t i);

void start_rx_uart();
int8_t uart_rx_ready();
char uart_rx_data();

#endif // UART_H
