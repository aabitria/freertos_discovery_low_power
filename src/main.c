/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;

// function prototypes
static void prvSetupHardware (void);
void printmsg (char *msg);
static void prvSetupUart (void);
void prvSetupGPIO (void);
void rtos_delay (uint32_t delay_in_ms);

TimerHandle_t timer_handle = NULL;

void vTask1_handler (void *params);
void vTask2_handler (void *params);
void vTask3_handler (void *params);

uint32_t sleep_mode_active = 0;

char usr_msg[250];

void timer_cb ( TimerHandle_t xTimer )
{
	//GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	if (sleep_mode_active == 0)
	{
	    vTaskSuspend(xTaskHandle1);
	    vTaskSuspend(xTaskHandle2);
	    vTaskSuspend(xTaskHandle3);
	    sleep_mode_active = 1;
	}
}


int main(void)
{
	// Enable DWT CYCCNT for Segger SYSVIEW
	DWT->CTRL |= (1 << 0);

	// Restore RCC settings to default (HSI)
	RCC_DeInit();

	// Update SystemCoreClock
	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg, "This is a demo of Task Notify APIs\r\n");
	printmsg(usr_msg);

	// Start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	// Let's create led task
	xTaskCreate(vTask1_handler, "TASK-1", 500, NULL, 3, &xTaskHandle1);

	xTaskCreate(vTask2_handler, "TASK-2", 500, NULL, 3, &xTaskHandle2);
	xTaskCreate(vTask3_handler, "TASK-3", 500, NULL, 3, &xTaskHandle3);

	timer_handle = xTimerCreate("LED-TIMER", pdMS_TO_TICKS(3000), pdFALSE, NULL, timer_cb);

	xTimerReset(timer_handle, portMAX_DELAY);

	// Start the scheduler
	vTaskStartScheduler();

	for(;;);
}

void vTask1_handler (void *params)
{
	while (1)
	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		rtos_delay(100);
		//vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void vTask2_handler (void *params)
{
	while (1)
	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		rtos_delay(50);
		//vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void vTask3_handler (void *params)
{
	while (1)
	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
		rtos_delay(50);
		//vTaskDelay(pdMS_TO_TICKS(20));
	}
}

static void prvSetupUart (void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart2_pins;

	// 1. Enable UART Clock; but which UARTx?  Must use PA2/PA3 pins - USART2
	// USART2 is connected to APB1 bus
	// Enable GPIOA clk as well
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// PA2 - UART_TX; PA3 - UART_RX
	// PA2/3 are in GPIOA, functioning as USART2 if cfg'ed as AF7
	// 2. Configure GPIOA to configure PA2/3 as UART pins (Alternate function config)
	memset(&gpio_uart_pins, 0, sizeof(gpio_uart_pins));
	gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpio_uart_pins);

	// 3. AF settings for MCU pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// 4. UART parameter initialization
	memset(&uart2_pins, 0, sizeof(uart2_pins));
	uart2_pins.USART_BaudRate = 115200;
	uart2_pins.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart2_pins.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	uart2_pins.USART_Parity = USART_Parity_No;
	uart2_pins.USART_StopBits = USART_StopBits_1;
	uart2_pins.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &uart2_pins);

	// 5. Enable UART2
	USART_Cmd(USART2, ENABLE);
}

// All MCU/board related initialization
static void prvSetupHardware (void)
{
	// Setup GPIO
	prvSetupGPIO();

	// Setup UART2
	prvSetupUart();
}

void printmsg (char *msg)
{
	uint32_t i;
	uint32_t count = strlen(msg);

	for (i = 0; i < count; i++)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET);
		USART_SendData(USART2, msg[i]);
	}

	//while ( USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
}

void prvSetupGPIO (void)
{
	GPIO_InitTypeDef led_init, button_init;
	EXTI_InitTypeDef exti_init;

	// Enable peripheral clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Use pin PD11 as another output for tick hook and PD10 for idle hook
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_Pin = (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	led_init.GPIO_Speed = GPIO_Low_Speed;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &led_init);

	button_init.GPIO_Mode = GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_0;
	button_init.GPIO_Speed = GPIO_Low_Speed;
	button_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &button_init);

	// interrupt configuration for the button (PA0) - EXTI_0
	// 1. system configuration for EXTI line
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	// 2. Initialize EXTI regs
	exti_init.EXTI_Line = EXTI_Line0;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	// 3. NVIC configuration; EXTI_0 is connected to IRQ line #6; set priority and enable
	NVIC_SetPriority(EXTI0_IRQn, 5);
	NVIC_EnableIRQ(EXTI0_IRQn);

}

void button_handler (void *params)
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);

	// Reset the timer
	if (xTimerResetFromISR(timer_handle, pdFALSE) != pdPASS)
	{
		while(1);
	}

	// If sleep flag not set as expected, leave
	if (sleep_mode_active == 0)
	{
		return;
	}

	// We all know that sleep flag is active and set, so act to unblock the tasks
	if (xTaskResumeFromISR(xTaskHandle1) != pdPASS)
	{
		while(1);
	}

	if (xTaskResumeFromISR(xTaskHandle2) != pdPASS)
	{
		while(1);
	}

	if (xTaskResumeFromISR(xTaskHandle3) != pdPASS)
	{
		while(1);
	}

	sleep_mode_active = 0;
}

void EXTI0_IRQHandler (void)
{
	// To enable SEGGER to trace interrupts
	traceISR_ENTER();

	// 1. Clear the interrupt pending bit in EXTI0
	EXTI_ClearITPendingBit(EXTI_Line0);
	button_handler(NULL);

	traceISR_EXIT();
}

void rtos_delay (uint32_t delay_in_ms)
{
	uint32_t tick_count_local = 0;
	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ) / 1000;

	tick_count_local = xTaskGetTickCount();
	while ( xTaskGetTickCount() < (tick_count_local + delay_in_ticks) );
}

void vApplicationIdleHook (void)
{
	// set to high before sleeping to see if sleeping will hold this value or not
	// for oscilloscope verification
	//GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_SET);

	// Send CPU to normal sleep
	__WFI();

	// set to low
	//GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_RESET);
}

void vApplicationTickHook (void)
{
	// Toggle PD11 for observation in oscilloscope
	GPIO_ToggleBits(GPIOD, GPIO_Pin_11);
	//GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_SET);
	//GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_RESET);
}

void vPreSleepProcessing (uint32_t xModifiableIdleTime)
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);

	// Setup for stop mode
	PWR_EnterSTOPMode(PWR_LowPowerRegulator_ON, PWR_STOPEntry_WFI);
}

void vPostSleepProcessing (uint32_t xModifiableIdleTime)
{
	SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);

	// Update SystemCoreClock
	//SystemCoreClockUpdate();
}
