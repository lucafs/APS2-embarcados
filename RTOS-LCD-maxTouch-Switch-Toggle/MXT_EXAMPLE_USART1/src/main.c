#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "maxTouch/maxTouch.h"
#include "tfont.h"
#include "digital521.h"
#include "icons/BScreen.h"

typedef struct
{
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
}
calendar;

/************************************************************************/
/*prototypes                                                           */
/************************************************************************/
void but1_callback(void);

/************************************************************************/
/*LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES 10

struct ili9488_opt_t g_ili9488_display_opt;
//BUTTONS
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH / 2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT / 2;
const uint32_t BUTTON_NIGHT_W_H = 40;
const uint32_t BUTTON_PPS_W = 65;
const uint32_t BUTTON_H_PAUSE_STOP = 65;
const uint32_t BUTTON_H_PLAY = 65;
//Locations
const uint32_t LOCX_PAUSE = 40;
const uint32_t LOCX_PLAY = 130;
const uint32_t LOCX_STOP = 250;
const uint32_t LOCX_NIGHT = 255;
const uint32_t LOCY_PPS = 415;
const uint32_t LOCY_NIGHT = 10;
const uint32_t LOCX_INFO1 = 15;
const uint32_t LOCX_INFO2 = 175;
const uint32_t LOCY_INFO1 = 65;
const uint32_t LOCY_INFO2 = 205;
const uint32_t LOCY_INFO3 = 315;


#define BUT_PIO PIOA
#define BUT_PIO_ID 10
#define BUT_PIO_IDX 11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)

char horario[512];
char cronometro[512];
char velocidade_atualC[512];
char velocidade_mediaC[512];
char distanciaC[512];
char aceleracaoC[512];
float pi = 3.14159265359;
float M_TO_KM = 0.001;
float Ms_TO_KMh = 3.6;
float raio = 0.66;	//bike de aro 26
volatile char velocidade_atual = 0.0;
volatile char aceleracao_atual = 0.0;
volatile char velocidade_media = 0.0;
volatile char comecou_percurso = 0;
volatile char distancia = 0.0;
volatile char reset = 0;
volatile char minutos = 0;
volatile char segundos = 0;
volatile Bool f_rtt_alarme = false;
volatile char flag_5sec = 0;
volatile char numero_rotacoes = 0;
volatile char pause = 1;
SemaphoreHandle_t ButSemaphore;
SemaphoreHandle_t RttSemaphore;
SemaphoreHandle_t RtcSemaphore;

//flag para mostrar que tempo passou
volatile char rtt_irq = 0;
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/*Botoes lcd                                                           */
/************************************************************************/

/************************************************************************/
/*RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE (2 * 1024 / sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_CALC_STACK_SIZE (5 * 1024 / sizeof(portSTACK_TYPE))
#define TASK_CALC_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_LCD_STACK_SIZE (4 * 1024 / sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY (tskIDLE_PRIORITY)

typedef struct
{
	uint x;
	uint y;
}
touchData;

QueueHandle_t xQueueTouch;

/************************************************************************/
/*handler/callbacks                                                    */
/************************************************************************/
void RTT_Handler(void)
{
	uint32_t ul_status;

	/*Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/*IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {}

	/*IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(RttSemaphore, &xHigherPriorityTaskWoken);
		f_rtt_alarme = true;	// flag RTT alarme
	}
}
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(RtcSemaphore, &xHigherPriorityTaskWoken);
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}

	/*Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM)
	{
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	}

	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/*RTOS hooks                                                           */
/************************************************************************/

/**
 *\brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
	signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR*) pcTaskName);
	/*If the parameters have been corrupted then inspect pxCurrentTCB to
	 *identify which task has overflowed its stack.
	 */
	for (;;) {}
}

/**
 *\brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 *\brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void)
{
	/*Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/*Force an assert. */
	configASSERT((volatile void *) NULL);
}

/************************************************************************/
/*init                                                                 */
/************************************************************************/
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/*Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT))
	;

	rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);

	/*Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type)
{
	/*Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/*Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/*Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/*Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/*Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc, irq_type);
}

static void configure_lcd(void)
{
	/*Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/*Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/************************************************************************/
/*funcoes                                                              */
/************************************************************************/

void draw_screen(void)
{
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH - 1, ILI9488_LCD_HEIGHT - 1);
}

void draw_button(uint32_t clicked)
{
	static uint32_t last_state = 255;	// undefined
	if (clicked == last_state) return;

	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(BUTTON_X - BUTTON_W / 2, BUTTON_Y - BUTTON_H / 2, BUTTON_X + BUTTON_W / 2, BUTTON_Y + BUTTON_H / 2);
	if (clicked == 1)
	{
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
		ili9488_draw_filled_rectangle(BUTTON_X - BUTTON_W / 2 + BUTTON_BORDER, BUTTON_Y + BUTTON_BORDER, BUTTON_X + BUTTON_W / 2 - BUTTON_BORDER, BUTTON_Y + BUTTON_H / 2 - BUTTON_BORDER);
	}
	else if (clicked == 0)
	{
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
		ili9488_draw_filled_rectangle(BUTTON_X - BUTTON_W / 2 + BUTTON_BORDER, BUTTON_Y - BUTTON_H / 2 + BUTTON_BORDER, BUTTON_X + BUTTON_W / 2 - BUTTON_BORDER, BUTTON_Y - BUTTON_BORDER);
	}
	else if (clicked == 2)
	{
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_YELLOW));
		ili9488_draw_filled_rectangle(BUTTON_X - BUTTON_W / 2 + BUTTON_BORDER, BUTTON_Y - BUTTON_H / 2 + BUTTON_BORDER, BUTTON_X + BUTTON_W / 2 - BUTTON_BORDER, BUTTON_Y - BUTTON_BORDER);
	}
	last_state = clicked;
}

uint32_t convert_axis_system_x(uint32_t touch_y)
{
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH *touch_y / 4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x)
{
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT *touch_x / 4096;
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing)
{
	char *p = text;
	while (*p != NULL)
	{
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if (letter <= font->end_char)
		{
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
	/*USART tx buffer initialized to 0 */
	uint8_t i = 0; /*Iterator */

	/*Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/*first touch only */
	uint first = 0;

	/*Collect touch events and put the data in a string,
	 *maximum 2 events at the time */
	do {

		/*Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK)
		{
			continue;
		}

		/************************************************************************/
		/*Envia dados via fila RTOS                                            */
		/************************************************************************/
		if (first == 0)
		{ *x = convert_axis_system_x(touch_event.y);
			*y = convert_axis_system_y(touch_event.x);
			first = 1;
		}

		i++;

		/*Check if there is still messages in the queue and
		 *if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) &(i < MAX_ENTRIES));
}

/************************************************************************/
/*tasks                                                                */
/************************************************************************/

void task_mxt(void)
{

	struct mxt_device device; /*Device data container */
	mxt_init(&device); /*Initialize the mXT touch device */
	touchData touch; /*touch queue data type*/

	while (true)
	{
		/*Check for any pending messages and run message handler if any
		 *message is found in the queue */
		if (mxt_is_message_pending(&device))
		{
			mxt_handler(&device, &touch.x, &touch.y);
			xQueueSend(xQueueTouch, &touch, 0); /*send mesage to queue */
			vTaskDelay(200);

			// limpa touch
			while (mxt_is_message_pending(&device))
			{
				mxt_handler(&device, NULL, NULL);
				vTaskDelay(50);
			}
		}

		vTaskDelay(300);
	}
}
void but_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// 	printf("but_callback \n");
	xSemaphoreGiveFromISR(ButSemaphore, &xHigherPriorityTaskWoken);
	numero_rotacoes += 1;
}
void butInit(void)
{

	NVIC_SetPriority(BUT_PIO_ID, 4);	// Prioridade 4
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but_callback);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUT_PIO_ID);

}
void calcParametros(volatile char numero_rotacoes)
{
	int numero_rotacoes_antigo = 0;
	float distancia_antiga = distancia;
	float distancia_atual = 2 *pi *raio * numero_rotacoes + distancia;
	distancia += 2 *pi *raio * numero_rotacoes;
	float velocidade_angular = 2 *pi *numero_rotacoes*(1/5);
	float velocidade_antiga = velocidade_atual;
	velocidade_atual = raio *velocidade_angular *Ms_TO_KMh;
	velocidade_media = velocidade_media + velocidade_atual;
	//talvez seja bom colocar um ganho
	aceleracao_atual = (velocidade_atual - velocidade_antiga) / 5;
	numero_rotacoes_antigo = numero_rotacoes;
}

int set_touch(uint32_t tx, uint32_t ty, uint32_t LocX, uint32_t LocY, uint32_t ButtonW, uint32_t ButtonH)
{
	if (tx >= LocX && tx <= LocX + ButtonW)
	{
		if (ty >= LocY && ty <= LocY + ButtonH)
		{
			return 1;
		}
	}
	return 0;
}

void update_screen(uint32_t tx, uint32_t ty)
{

	if (set_touch(tx, ty, LOCX_PAUSE, LOCY_PPS, BUTTON_PPS_W, BUTTON_H_PAUSE_STOP))
	{

		pause = 0;
		reset = 0;
	}
	if (set_touch(tx, ty, LOCX_PLAY, LOCY_PPS, BUTTON_PPS_W, BUTTON_H_PLAY))
	{

		pause = 1;
		comecou_percurso = 1;
		reset = 0;
	}
	if (set_touch(tx, ty, LOCX_STOP, LOCY_PPS, BUTTON_PPS_W, BUTTON_H_PAUSE_STOP))
	{
		reset = 1;
	}

	//if(tx >= BUTTON_X-BUTTON_W/2 && tx <= BUTTON_X + BUTTON_W/2) {
	//  if(ty >= BUTTON_Y-BUTTON_H/2 && ty <= BUTTON_Y) {
	//    draw_button(1);
	//    } else if(ty > BUTTON_Y && ty < BUTTON_Y + BUTTON_H/2) {
	//    draw_button(0);
	//  }
	//}
}


void task_lcd(void)
 {
	xQueueTouch = xQueueCreate(10, sizeof(touchData));
	configure_lcd();

	draw_screen();

	// Escreve DEMO - BUT no LCD
	// desenha imagem lavagem na posicao X=80 e Y=150
	ili9488_draw_pixmap(0, 0, BScreen.width, BScreen.height, BScreen.data);
	
	// INformações na tela
	font_draw_text(&digital52, "00:00", LOCX_INFO1, LOCY_INFO1, 1);
	font_draw_text(&digital52, "00.00", LOCX_INFO2, LOCY_INFO1, 1);
	font_draw_text(&digital52, "00.00", LOCX_INFO1, LOCY_INFO2, 1);
	font_draw_text(&digital52, "00.00", LOCX_INFO2, LOCY_INFO2, 1);
	font_draw_text(&digital52, "00.00", LOCX_INFO1, LOCY_INFO3, 1);
	font_draw_text(&digital52, "00:00", LOCX_INFO2, LOCY_INFO3, 1);
	
	//horas
	int hh = 00;
	int mm = 00;
	int ss = 00;
	
	//count while
	int while_count = 0;

	// strut local para armazenar msg enviada pela task do mxt
	touchData touch;

	///////////////////////////////////////TASK_CALC///////////////////
	//AO ADICIONAR BOTAO DE START E PAUSE PAUSE =0 e Start -> mecher em comecou percurso
	ButSemaphore = xSemaphoreCreateBinary();
	RttSemaphore = xSemaphoreCreateBinary();
	RtcSemaphore = xSemaphoreCreateBinary();

	butInit();
	f_rtt_alarme = true;
	sprintf(cronometro, "%2d:%2d", minutos, segundos);
	pause = 0;
	comecou_percurso = 0;
	reset = 0;
	if (ButSemaphore == NULL)
	{
		printf("falha em criar o semaforo botao \n");
	}
	if (RttSemaphore == NULL)
	{
		printf("falha em criar o semaforo RTT \n");
	}
	if (RtcSemaphore == NULL)
	{
		printf("falha em criar o semaforo RTC \n");
	}

	/**Configura RTC */
	// 	calendar rtc_initial = {2020, 5, 16, 15, 16, 34, 1};
	// 	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	// 	uint32_t hh, mm, ss;
	// 	/*configura alarme do RTC */
	// 	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	// 	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	/////////////////////////////////////////////////////////////////////////////

	while (true)
	{

		if (xQueueReceive(xQueueTouch, &(touch), (TickType_t) 500 / portTICK_PERIOD_MS))
		{
			update_screen(touch.x, touch.y);
			printf("x:%d y:%d\n", touch.x, touch.y);
		}

		if (reset)
		{
			printf("Reset");
			comecou_percurso = 0;
			segundos = 0;
			minutos = 0;
			flag_5sec = 0;
			distancia = 0.0;
			velocidade_atual = 0.0;
			velocidade_media = 0.0;
			aceleracao_atual = 0.0;
			reset = 0;
		}
		//if (xSemaphoreTake(ButSemaphore, (TickType_t) 500) == pdTRUE)
		//{
			// 			numero_rotacoes+=1;
		//}
		if (xSemaphoreTake(RtcSemaphore, (TickType_t) 500) == pdTRUE)
		{
			rtc_get_time(RTC, &hh, &mm, &ss);
			sprintf(horario, "%2d:%2d", hh, mm);
		}		

		if (f_rtt_alarme)
		{

			/*
			 *IRQ apos 4s -> 8 * 0.5
			 */
			uint16_t pllPreScale = (int)(((float) 32768) / 5.0);
			uint32_t irqRTTvalue = 2;

			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);

			f_rtt_alarme = false;
		}
		if (flag_5sec == 5)
		{
			calcParametros(numero_rotacoes);
			//printf("numero de rotacoes = %d \n", numero_rotacoes);
			//printf("vel media = %d \n", velocidade_media);
			sprintf(distanciaC, "%2f", distancia);
			sprintf(velocidade_atualC, "%2f", velocidade_atual);
			sprintf(velocidade_mediaC, "%2f", velocidade_media/while_count);
			sprintf(aceleracaoC, "%2f", aceleracao_atual);
			numero_rotacoes = 0;
			flag_5sec = 0;
		}
		//passagem de tempo do cronometro
		if (xSemaphoreTake(RttSemaphore, (TickType_t) 500) == pdTRUE && comecou_percurso)
		{
			if (pause)
			{
				printf("pause");
				//calculo dos parametros
				if (segundos > 59)
				{
					minutos += 1;
					segundos = 0;
					
				}
				else
				{
					segundos++;
					
				}
				flag_5sec += 1;
				// 				printf("tempo = %s \n", cronometro);
			}
			else
			{
				printf("play");
			}
			sprintf(cronometro, "%2d:%2d", minutos, segundos);
		}
		font_draw_text(&digital52, horario, LOCX_INFO1, LOCY_INFO1, 1);//Horário
		font_draw_text(&digital52, velocidade_atualC, LOCX_INFO2, LOCY_INFO1, 1);//Vel
		font_draw_text(&digital52, velocidade_mediaC, LOCX_INFO1, LOCY_INFO2, 1);//VelMed
		font_draw_text(&digital52, aceleracaoC, LOCX_INFO2, LOCY_INFO2, 1);//Acele
		font_draw_text(&digital52, distanciaC, LOCX_INFO1, LOCY_INFO3, 1);//Dist
		font_draw_text(&digital52, cronometro, LOCX_INFO2, LOCY_INFO3, 1);//Crono
		printf(cronometro);
		/////////////////////////////////////////////////////////
		while_count ++;
	}
}

/************************************************************************/
/*main                                                                 */
/************************************************************************/

int main(void)
{
	/*Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = { .baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /*Initialize system clocks */
	board_init(); /*Initialize board */

	/*Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	/*Create task to handler touch */
	if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create mxt task\r\n");
	}

	/*Create task to handler LCD */
	if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create lcd task\r\n");
	}

	//if (xTaskCreate(task_calc, "calc", TASK_CALC_STACK_SIZE, NULL, TASK_CALC_STACK_PRIORITY, NULL) != pdPASS)
	//{
	//	printf("Failed to create calc task\r\n");
	//}

	/*Start the scheduler. */
	vTaskStartScheduler();

	while (1) {}

	return 0;
}