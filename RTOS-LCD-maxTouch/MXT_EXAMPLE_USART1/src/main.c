#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "tfont.h"
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "tfont.h"
#include "digital521.h"
#include "ar.h"
#include "soneca.h"
#include "termometro.h"

/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_AFEC_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_AFEC_STACK_PRIORITY        (tskIDLE_PRIORITY)

/************************************************************************/
/* PIOS OLED                                                                 */
/************************************************************************/
//butao 1 oled
#define EBUT1_PIO PIOD // EXT 9 PD28
#define EBUT1_PIO_ID 16
#define EBUT1_PIO_IDX 28u
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)
//butao 2 oled
#define EBUT2_PIO PIOA //  Ext 4 PA19 PA = 10
#define EBUT2_PIO_ID 10
#define EBUT2_PIO_IDX 19u
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0


/************************************************************************/
/* AFEC                                                             */
/************************************************************************/

/** Reference voltage for AFEC,in mv. */
#define VOLT        (3300) 
/* Canal do sensor de temperatura */
#define AFEC_CHANNEL 0
//PD30
#define AFEC_CHANNEL_POT_SENSOR 0 

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES  3

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;


//Potenciometro

//Volatile pela interrupcao do hardware
volatile uint32_t potencio_ul_value;

volatile uint32_t potencio_convertida = 0;

// Ciclo Pwm

volatile uint32_t duty = 30;

// Buffer do potenciometro
char bufferPotencia[32];
// Buffer do PWM
char bufferMotor[32];

// Criar o canal do pwm para o LED

pwm_channel_t global_pwm_channel_led;

typedef struct {
	uint x;
	uint y;
} touchData;


QueueHandle_t xQueueTouch;

//Call back botao
void pwm_callback_add(void);
//Call back Pwm
void pwm_callback_sub(void);


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
	
	/** Semaforo a ser usado pelas tasks */
	SemaphoreHandle_t xSemaphorePwm_add; 
	SemaphoreHandle_t xSemaphorePwm_sub;


/************************************************************************/
/* init                                                                 */
/************************************************************************/


static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
* \brief Set maXTouch configuration
*
* This function writes a set of predefined, optimal maXTouch configuration data
* to the maXTouch Xplained Pro.
*
* \param device Pointer to mxt_device struct
*/
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
	MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	* the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	* value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void io_init(void){
	// Pmc para os PIOS
	pmc_enable_periph_clk(EBUT1_PIO_ID);
	pmc_enable_periph_clk(EBUT2_PIO_ID);

	//Configura o pino como botao com pull-up
	pio_configure(EBUT1_PIO, PIO_INPUT, EBUT1_PIO_IDX_MASK, PIO_DEBOUNCE|PIO_PULLUP);
	pio_configure(EBUT2_PIO, PIO_INPUT, EBUT2_PIO_IDX_MASK, PIO_DEBOUNCE|PIO_PULLUP);
	
	// a função de callback é a: butt_callback() e do segundo botao pwm_callback();
	pio_handler_set(EBUT1_PIO,
	EBUT1_PIO_ID,
	EBUT1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	pwm_callback_add);
	
	pio_handler_set(EBUT2_PIO,
	EBUT2_PIO_ID,
	EBUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	pwm_callback_sub);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(EBUT1_PIO_ID);
	NVIC_SetPriority(EBUT1_PIO_ID, 5);
	
	NVIC_EnableIRQ(EBUT2_PIO_ID);
	NVIC_SetPriority(EBUT2_PIO_ID, 5);
	
	// Ativa interrupção
	pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
	pio_enable_interrupt(EBUT2_PIO, EBUT2_PIO_IDX_MASK);

	
}


void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	global_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	global_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	global_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	global_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	global_pwm_channel_led.ul_duty = duty;
	global_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM0, &global_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}

/************************************************************************/
/* funcoes e Callbacks                                                   */
/************************************************************************/

void pwm_callback_add(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphorePwm_add, &xHigherPriorityTaskWoken);
}

void pwm_callback_sub(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphorePwm_sub, &xHigherPriorityTaskWoken);
}


static void AFEC_callback(void)
{
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	afec_start_software_conversion(AFEC0);
	potencio_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT_SENSOR);
}

static void config_ADC(void){
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_callback, 1);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC0, AFEC_CHANNEL);
}


//Essa funcao fara a regra de 3 da voltagem para saber a conversao em bits - NAO EH UMA TASK
static int32_t convertAnalogic(int32_t ADC_value){
	
	int32_t val_vol;
	int32_t val_temp;
	
	/* converte bits -> tensão (Volts)
   */
	val_vol = ADC_value * VOLT / (float) MAX_DIGITAL;
	val_vol = val_vol*100/3300;

	return(val_vol);
	
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

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	ili9488_draw_pixmap(20, 250, ar.width, ar.height, ar.data);
	font_draw_text(&digital52, "%", 140, 250, 1);
	ili9488_draw_pixmap(30, 380, termometro.width, termometro.height, termometro.data);
	font_draw_text(&digital52, "15", 110, 380, 1);
	ili9488_draw_pixmap(210, 20, soneca.width, soneca.height, soneca.data);
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

void update_screen() {
	ili9488_draw_filled_rectangle(100, 350,110,380);
	sprintf(bufferPotencia,"%d",potencio_convertida);
	font_draw_text(&digital52, bufferPotencia, 110, 380, 1);
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
		}
		vTaskDelay(100);
	}
}

void task_lcd(void){
	
	const TickType_t xDelay = 4000/ portTICK_PERIOD_MS;
	
	xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	configure_lcd();
	
	draw_screen();
	
	// Escreve HH:MM no LCD
	font_draw_text(&digital52, "HH:MM", 0, 0, 1);
	
	// Aumentar o Ciclo
	xSemaphorePwm_add = xSemaphoreCreateBinary();
	// Diminuir o Ciclo
	xSemaphorePwm_sub = xSemaphoreCreateBinary(); 
	
	// Aqui sera onde eu irei comnparar o XSemaphore para o buffer do pwm ser escrito, tanto se for aumentar, quanto pra condicao de diminuir
	// Ver se o xSemaphore eh nulo
	
	if (xSemaphorePwm_add == NULL)
	printf("falha em criar o semaforo \n");

	pwm_channel_update_duty(PWM0, &global_pwm_channel_led, 100-duty);

	PWM0_init(0, duty);
	
	io_init();

	
	while (true) {
		update_screen();
			// Esta condicao ira atualizar o valor do ciclo aumentando-o de 10 a cada clicada 
			if( xSemaphoreTake(xSemaphorePwm_add, ( TickType_t ) 500) == pdTRUE ){
				if (duty<100){
					duty+=10;
				}
				pwm_channel_update_duty(PWM0, &global_pwm_channel_led, 100-duty);
				ili9488_draw_filled_rectangle(110, 250, 130, 290);
				sprintf(bufferPotencia, "%d", duty);
				font_draw_text(&digital52, bufferPotencia, 110, 250, 1);
			}
			
		}
	}

void taskPotencio(void *pvParameters){
	//Setar para 4 segundos
	const TickType_t xDelay = 4000 / portTICK_PERIOD_MS;
	UNUSED(pvParameters);
	while (true) {
		AFEC_callback();
		//atualizar o valor do potenciometro usando a funcao de conversao
		potencio_convertida = convertAnalogic(potencio_ul_value);
		vTaskDelay(xDelay);
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
	ioport_init();
	
	/* inicializa e configura adc */
	config_ADC();
	
	afec_start_software_conversion(AFEC0);
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	
	/* Create task to handler touch */
	if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to handler LCD */
	if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to AFEC converter */
	if (xTaskCreate(taskPotencio, "afec", TASK_AFEC_STACK_SIZE, NULL, TASK_AFEC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* incializa conversao ADC */
	afec_start_software_conversion(AFEC0);

	while(1){

	}

	return 0;
}
