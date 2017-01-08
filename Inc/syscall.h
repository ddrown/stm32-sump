#ifndef SYSCALL_H
#define SYSCALL_H

extern uint32_t timeouts;
int _write(int fd, const void *buf, size_t count);
caddr_t _sbrk(int incr);
uint32_t heapsize();
uint32_t heapmax();
uint32_t stacksize();
uint32_t stackmax();

#endif // SYSCALL_H
