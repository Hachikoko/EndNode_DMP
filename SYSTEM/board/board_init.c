#include "stm32f4xx.h"
#include "uart.h"
#include "Timer.h"
#include "sys.h"
#include "gpio.h"
#include "i2c.h"


#define TICK_FREQ (1000000u)
volatile uint32_t g_ul_ms_ticks=0;
static volatile uint32_t TimingDelay=0;
unsigned long idle_time=0;
extern uint32_t SystemCoreClock; //168000000=168Mhz (original value)

void board_init(){
	SystemCoreClockUpdate();          
	if (SysTick_Config (SystemCoreClock / TICK_FREQ)) {     // Setup SysTick Timer for 1 msec interrupts
		while (1);                                          // Handle Error
	}	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断优先级分组2
	
	Set_I2C_Retry(5);
	
	/* Enable PWR APB1 Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to Backup */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset RTC Domain */
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
	
	//Configure Interrupts
	GPIO_Config();  
	
	I2cMaster_Init(); 
	
	USART_Config();
	
}

void udelay(unsigned long nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void mdelay(unsigned long nTime)
{
	udelay((1000u) * nTime);
}

int get_tick_count(unsigned long *count)
{
        count[0] = (g_ul_ms_ticks/1000u);
	return 0;
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
		TimingDelay--;
}

void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}

void SysTick_Handler(void)
{
  	TimingDelay_Decrement();
    TimeStamp_Increment();
}

