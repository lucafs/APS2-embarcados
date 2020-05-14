#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "maxTouch/maxTouch.h"
#include "tfont.h"
#include "digital521.h"

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void but1_callback(void);


/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        10

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

#define BUT_PIO			  PIOA
#define BUT_PIO_ID		  10
#define BUT_PIO_IDX		  11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)


char cronometro[512];
float pi =3.14159265359;
float raio =0.66;//bike de aro 26
volatile char velocidade_atual=0;
volatile char aceleracao_atual=0;
volatile char velocidade_media=0;
volatile char distancia=0;
volatile char comecou_percurso = 0;
volatile char reset = 0;
volatile char minutos = 0;
volatile char segundos = 0;
volatile Bool f_rtt_alarme = false;
volatile char flag_4sec =0;
volatile char numero_rotacoes=0;
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t RttSemaphore;

//flag para mostrar que tempo passou
volatile char rtt_irq = 0;
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);


/************************************************************************/
/* Botoes lcd                                                           */
/************************************************************************/


/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_CALC_STACK_SIZE            (5*1024/sizeof(portSTACK_TYPE))
#define TASK_CALC_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct {
  uint x;
  uint y;
} touchData;

QueueHandle_t xQueueTouch;

/************************************************************************/
/* handler/callbacks                                                    */
/************************************************************************/
void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
	{
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		printf("but_callback \n");
		xSemaphoreGiveFromISR(RttSemaphore, &xHigherPriorityTaskWoken);	
		f_rtt_alarme = true; // flag RTT alarme
	}
}

/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
  * identify which task has overflowed its stack.
  */
  for (;;) {
  }
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
  pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT))
	;

	rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

static void configure_lcd(void){
  /* Initialize display parameter */
  g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
  g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
  g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
  g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

  /* Initialize LCD */
  ili9488_init(&g_ili9488_display_opt);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void draw_screen(void) {
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void draw_button(uint32_t clicked) {
  static uint32_t last_state = 255; // undefined
  if(clicked == last_state) return;
  
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
  ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
  if(clicked) {
    ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
    ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
    } else {
    ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
    ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
  }
  last_state = clicked;
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
  // entrada: 4096 - 0 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
  // entrada: 0 - 4096 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void update_screen(uint32_t tx, uint32_t ty) {
  if(tx >= BUTTON_X-BUTTON_W/2 && tx <= BUTTON_X + BUTTON_W/2) {
    if(ty >= BUTTON_Y-BUTTON_H/2 && ty <= BUTTON_Y) {
      draw_button(1);
      } else if(ty > BUTTON_Y && ty < BUTTON_Y + BUTTON_H/2) {
      draw_button(0);
    }
  }
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
  char *p = text;
  while(*p != NULL) {
    char letter = *p;
    int letter_offset = letter - font->start_char;
    if(letter <= font->end_char) {
      tChar *current_char = font->chars + letter_offset;
      ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
      x += current_char->image->width + spacing;
    }
    p++;
  }
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
  /* USART tx buffer initialized to 0 */
  uint8_t i = 0; /* Iterator */

  /* Temporary touch event data struct */
  struct mxt_touch_event touch_event;
  
  /* first touch only */
  uint first = 0;

  /* Collect touch events and put the data in a string,
  * maximum 2 events at the time */
  do {

    /* Read next next touch event in the queue, discard if read fails */
    if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
      continue;
    }
    
    /************************************************************************/
    /* Envia dados via fila RTOS                                            */
    /************************************************************************/
    if(first == 0 ){
      *x = convert_axis_system_x(touch_event.y);
      *y = convert_axis_system_y(touch_event.x);
      first = 1;
    }
    
    i++;

    /* Check if there is still messages in the queue and
    * if we have reached the maximum numbers of events */
  } while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
  
  struct mxt_device device; /* Device data container */
  mxt_init(&device);       	/* Initialize the mXT touch device */
  touchData touch;          /* touch queue data type*/
  
  while (true) {
    /* Check for any pending messages and run message handler if any
    * message is found in the queue */
    if (mxt_is_message_pending(&device)) {
      mxt_handler(&device, &touch.x, &touch.y);
      xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
      vTaskDelay(200);
      
      // limpa touch
      while (mxt_is_message_pending(&device)){
        mxt_handler(&device, NULL, NULL);
        vTaskDelay(50);
      }
    }
    
    vTaskDelay(300);
  }
}
void but_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}
void butInit(void){
	
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but_callback);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);
		NVIC_EnableIRQ(BUT_PIO_ID);

}
void calcParametros(volatile char numero_rotacoes){
	velocidade_media=numero_rotacoes/4;
	int velocidade_angular=(2*pi*numero_rotacoes)/4;
	int velocidade_antiga=velocidade_atual;
	velocidade_atual=raio*velocidade_angular;
	distancia+=2*pi*raio*numero_rotacoes;
	//talvez seja bom colocar um ganho
	aceleracao_atual=(velocidade_atual-velocidade_antiga)/4;
}
void task_calc(void){
	//AO ADICIONAR BOTAO DE START E PAUSE EM comecou percurso
	xSemaphore = xSemaphoreCreateBinary();
	RttSemaphore = xSemaphoreCreateBinary();
	butInit();
	f_rtt_alarme = true;
	sprintf(cronometro, "%2d:%2d", minutos, segundos);
	if (xSemaphore == NULL){
	  printf("falha em criar o semaforo \n");}
	if (RttSemaphore == NULL){
		printf("falha em criar o semaforo RTT \n");}

  while (true) {
		if(reset){
			comecou_percurso=0;
			segundos=0;
			minutos=0;
			flag_4sec=0;
			distancia=0;
			velocidade_atual=0;
			velocidade_media=0;
			aceleracao_atual=0;
			distancia=0;
		}
	  		if (f_rtt_alarme)
		{

			/*
		   * IRQ apos 4s -> 8*0.5
		   */
			uint16_t pllPreScale = (int)(((float)32768) / 4.0);
			uint32_t irqRTTvalue = 2;

			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);

			f_rtt_alarme = false;
		}
		if(flag_4sec==4){
			calcParametros(numero_rotacoes);
			printf("velocidade atual = %d", velocidade_atual);
			numero_rotacoes=0;
			flag_4sec=0;
		}
		//passagem de tempo do cronometro
		if ( xSemaphoreTake(RttSemaphore, ( TickType_t ) 500) == pdTRUE  && comecou_percurso)
		{
			if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE ){
				numero_rotacoes+=1;
			}
			//calculo dos parametros
			if (segundos > 0)
			{

				segundos -= 1;
				sprintf(cronometro, "%2d:%2d", minutos, segundos);
			}
			if (segundos == 0 && minutos > 0)
			{
				segundos = 59;
				minutos -= 1;
				sprintf(cronometro, "%2d:%2d", minutos, segundos);
			}
			if (segundos == 0 && minutos == 0)
			{
				sprintf(cronometro, "%2d:%2d", minutos, segundos);
				comecou_percurso = 0;
			}
			flag_4sec+=1;
		}
    vTaskDelay(300);

    }

}





void task_lcd(void){
  xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
  configure_lcd();
  
  draw_screen();
  draw_button(0);
  
  // Escreve DEMO - BUT no LCD
  
  // strut local para armazenar msg enviada pela task do mxt
  touchData touch;
  
  while (true) {
    if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
      update_screen(touch.x, touch.y);
      printf("x:%d y:%d\n", touch.x, touch.y);
    }
  }
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
  /* Initialize the USART configuration struct */
  const usart_serial_options_t usart_serial_options = {
    .baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
    .charlength   = USART_SERIAL_CHAR_LENGTH,
    .paritytype   = USART_SERIAL_PARITY,
    .stopbits     = USART_SERIAL_STOP_BIT
  };

  sysclk_init(); /* Initialize system clocks */
  board_init();  /* Initialize board */
  
  /* Initialize stdio on USART */
  stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
  
  /* Create task to handler touch */
  if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create mxt task\r\n");
  }
  
  
  /* Create task to handler LCD */
  if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create lcd task\r\n");
  }
  
   	if (xTaskCreate(task_calc, "calc", TASK_CALC_STACK_SIZE, NULL, TASK_CALC_STACK_PRIORITY, NULL) != pdPASS) {
   		printf("Failed to create calc task\r\n");
  	}
  
  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){

  }


  return 0;
}