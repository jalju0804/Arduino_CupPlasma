//===================================================================
//                      CAR PLASMA STERILIZER
//
// FILE		: dwlc_20220305.c
// DATE		: March 5th, 2022
// CPU		: ATtiny24A @ IRC 8MHz
// COMPILER	: WinAVR - 20100110
// IDE      : Microchip Studio 7
// ISP		: unknown
//
// REVISION	: 	2022-03-05	initial version
//===================================================================

#define F_CPU 1000000UL // IRC 8MHz, CKSEL = “0010”, SUT = “10”, and CKDIV8

/*** INCLUDE(S) ***/
//#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

/*** CONSTANT(S) ***/
#define UV_EN		PB0 // Output, Active high
#define CHARGER_IN	PB1 // Input, HIGH=충전완료, LOW=충전중
#define FAN_EN		PB2

#define BLUELED_EN	PA1 // Output, Active high
#define REDLED_EN	PA2 // Output, Active high
#define ADC3_BAT	PA3 // Input(ad)
#define ERROR_F		PA4 // Output, Active high
#define FAN_EN2      PA5
#define DC18_EN		PA6 // Output, Active high
#define PLASMA_EN	PA7 // Output, Active high

//bit mask definition for system flag
#define FLAG_SYSTICK_bm		((unsigned char)(1<<0)) // 2.048msec tick // timer
#define FLAG_CHECK_TOUCHKEY_bm		((unsigned char)(1<<1)) // 
#define FLAG_KEYLOCK_bm		((unsigned char)(1<<2)) //key high check
#define FLAG_PLASMA_EVENT_bm			((unsigned char)(1<<3)) // 
#define FLAG_PLASMA_bm			((unsigned char)(1<<4)) // plasma 
#define FLAG_BIT5_bm			((unsigned char)(1<<5)) // x
#define FLAG_PWR_EVENT_bm		((unsigned char)(1<<6)) // 초기화 
#define FLAG_PWR_bm			((unsigned char)(1<<7)) // pwr on

#define N_CHAT_THRESH	32 // 2msec * 16 = 32msec
#define N_SLEEP_TIME	180

enum chargingmode { NO_CHARGING, NEED_CHARGING, DO_CHARGING };

#define N_PLASMA_LO_PERIOD		16000// 2sec	
#define N_PLASMA_LO_DUTY		100 // 기존 40
#define N_PLASMA_HI_PERIOD		500 // 0.25sec
#define N_PLASMA_HI_DUTY		200
#define N_PLASMA_OFF_PERIOD		1000 // 0.25sec
#define N_PLASMA_OFF_DUTY		100	

#define N_LED_PERIOD			8000 
#define N_LED_DUTY				4000	

#define N_POWER_ONOFF			100

#define N_BATLEVEL_3V3			675// VBAT = 3.5V 650 3.28V 
	
/*** VARIABLE(S) ***/
volatile unsigned char flag 	= 	0;

volatile unsigned char tim_base	=	0;

volatile unsigned char touch_chat	=	0;
volatile unsigned char touch_prev = 0;
volatile unsigned char touch_curr = 0;
volatile unsigned char touch_data = 0;

volatile unsigned char key_chat	=	0;
volatile unsigned char key_prev	=	0;
volatile unsigned char key_curr	=	0;

volatile unsigned char sys_mode = 2;
volatile unsigned char charger_mode = 0;

volatile uint16_t plasma_period = N_PLASMA_LO_PERIOD;
volatile uint16_t plasma_duty = N_PLASMA_LO_DUTY;
volatile uint16_t plasma_cnt = 0;

volatile uint16_t led_period = 0;
volatile uint16_t led_duty = 0;
volatile uint16_t led_cnt = 0;

volatile int power_touch_chat = 0;

/*** PROTOTYPE(S) ***/
void InitSystem(void);
void ReadAdcAndUpdateLed(void);
void HandleSystemTick(void);
void HandleSystem(void);
void HandleSystemPower(void);

/*** IMPLEMENTATION(S) ***/
void InitSystem(void)
{
	cli();

	DDRB  = (1<<PB0) | (1<<PB2);				//UV_EN=output
	PORTB = (0<<PB2) | (0<<PB0);	//UV_EN=off, TOUCH_IN=pull-up
	
	DDRA  = (1<<PA7) | (1<<PA6) | (1<<PA2) | (1<<PA1) | (1<<PA5); //PLASMA_EN, DC18_EN, REDLED_EN, BLUELED_EN, FAN_EN=output
	PORTA = (0<<PA7) | (0<<PA6) | (0<<PA2) | (0<<PA1) | (0<<PA5); //PLASMA_EN, DC18_EN, REDLED_EN, BLUELED_EN, FAN_EN=off
	
	DIDR0 = (1<<ADC3D);								// ADC3 Digital Input Disable
	ADMUX =  (1<<MUX1) | (1<<MUX0);		// Reference Vol : 1.1V, ADC3
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);	// ADC Frequency, /64
	ADCSRB = (0<<ADLAR);							// left adjusted & Free Running mode

	TCCR0A = (1<<WGM01) | (1<<WGM00);	// OC0x disconnected, fast PWM mode
	TCCR0B = (1<<CS01);					// CLKtn=CLKio/8 => 1MHz/8=125KHz (8usec)
	

	//GIMSK = (1<<PCIE1); // 1= 11 ~ 8 pcmsk1 0 = 7 ~ 0 pcmsk0
	//PCMSK1 = (1<<PCINT10);	// PCINT2 Enable
			// PCIE enable

	flag |= FLAG_PWR_bm;
	
	sei();
}

void ReadAdcAndUpdateLed(void)
{
	uint16_t value;
	
	if(flag & FLAG_PWR_bm) // 
	{
		ADCSRA |= (1<<ADSC);				// Start ADC Conversion
		while((ADCSRA & (1<<ADIF)) == 0);	// Wait for completion of ADC

		ADCSRA |= (1<<ADIF);				// Clear ADC Interrupt flag

		value = ADCL;
		value = ((ADCH)<<8)|value; 
		
		if( PINB & (1<<CHARGER_IN) )
		{
			charger_mode = DO_CHARGING;
			//PORTA  &= ~(1<<BLUELED_EN); // BLUELED Off
			PORTA  |= (1<<REDLED_EN); // REDLED On
			flag |= FLAG_PWR_bm;

		}
		else
		{
			charger_mode = NO_CHARGING;
			PORTA  &= ~(1<<REDLED_EN); // REDLED Off
			flag |= FLAG_PWR_bm;
			led_cnt=0;
		}
		
	}
}

void HandleSystem(void)
{
	flag |= FLAG_PLASMA_bm;
	flag |= FLAG_PWR_bm;
	flag |= FLAG_BIT5_bm;
	
	PORTB |= (1<<UV_EN); //UV ON
	PORTB |= (1<<FAN_EN); // FAN1 ON
	PORTA |= (1<<BLUELED_EN); //BLUE LED ON
	
	_delay_ms(1000);
	
	PORTA |= (1<<FAN_EN2); //BLUE LED + FAN2 ON
	
	_delay_ms(1000);
	
	PORTA |= (1<<DC18_EN); // DC18 On
	PORTA |= (1<<PLASMA_EN); // PLASMA On
	return;
}

void HandleSystemTick(void)
{
		//1. wake-up check routine

		//2. other timer services
		if( flag & FLAG_PLASMA_bm )
		{
			++plasma_cnt;
			if( plasma_duty == plasma_cnt )
			{
				PORTA  &= ~(1<<PLASMA_EN); // PLASMA Off
			} 	
			if( plasma_period == plasma_cnt )
			{
				PORTA  |= (1<<PLASMA_EN); // PLASMA On
				plasma_cnt = 0;
			}		
		}
		 		//3. other timer services`
// 		 if(flag & FLAG_BIT5_bm)
// 		 {
// 		 		switch(charger_mode)
// 		 		{
// 		 			case NO_CHARGING:
// 		 			PORTA &= ~(1<<REDLED_EN);
// 		 			break;
// 		 			case NEED_CHARGING:
// 		 			++led_cnt;
// 		 			if(led_duty == led_cnt)
// 					{
// 		 				PORTA &= ~(1<<REDLED_EN);
// 					}
// 					if( led_period == led_cnt )
// 		 			{
// 						PORTA  |= (1<<REDLED_EN); // REDLED On
// 		 				led_cnt = 0;
// 		 			}
// 		 			break;
// 		 			case DO_CHARGING:
// 		 			PORTA |= (1<<REDLED_EN);
// 		 			break;
// 		 		}
// 		 }
		return;
}
				

ISR(PCINT0_vect)
{
	//PORTB ^= (1<<PB3); //debug code, remove me
}



ISR(TIM0_OVF_vect)
{
	//PORTB ^= (1<<PB3); //debug code, remove me
}



ISR(WDT_vect)
{
	//PORTB ^= (1<<PB3); //debug code, remove me
	//PORTB &= ~(1<<PB3); //debug code, remove me
}



int main(void)
{
	InitSystem();
	HandleSystem();
	
	while(1)
	{
		ReadAdcAndUpdateLed();
		HandleSystemTick();
	}
	return 0;
}//main ends



/*** END-OF-FILE ***/



