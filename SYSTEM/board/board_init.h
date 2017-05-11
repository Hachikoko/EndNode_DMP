#ifndef __BOARD_INIT_H
#define __BOARD_INIT_H

void board_init();
int get_tick_count(unsigned long *count);
void mdelay(unsigned long nTime);
void udelay(unsigned long nTime);
void SysTick_Handler(void);

#endif	/* BOARD_INIT_H */