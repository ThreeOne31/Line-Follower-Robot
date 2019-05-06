//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            
//* DEV. BOARD:    UCT STM32 Development Board                       
//*==================================================================*
//* DESCRIPTION:                                                     *
//*   Line follower using PID controller                             *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
#define SW0 GPIO_IDR_0
#define GPIO_AF2 ((uint32_t)0x00000002)
//====================================================================
// GLOBAL VARIABLES
//====================================================================
uint32_t delay1 = 0;
uint32_t delay2 = 0;

//CONTROLLER VARIABLES
int position=2;			//state stead position in line with the middle sensor
int previous_pos= 2;
int Error = 0;
int Integral =0;
int Kp = 40;				// gain to magnify the error
int Ki = 80;				// Integral constant to improve tracking
int crossing =0;    		 //to counter number of perpendicular lines
//SPEED SETTING
int LmotorBaseSpeed = 24000;	// 36 000 corresponds to 75% duty circle 24 000 to 50%
int RmotorBaseSpeed = 24000;
int MaxSpeed = 36000;		//Max set at 75% duty circle,max can be set to 48 001 duty circle =1
char collection[16];

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ports(void);
void Delay (void);
void move(void);
void init_timer(void);
void ReadLine(void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();		// Initialise lcd
	init_ports();
	init_timer();
	GPIOB->ODR = 0b0110;

	while(1){
		Delay();				//Line sensor sampling time
		ReadLine();				//Read sensor positions
		if(crossing==3){			//end of line bring motor to stop
			TIM2->CCR3 =0; //duty cycle = ccr/(arr+1)   TIM2->ARR = 48000; // f = 1 KHz
			TIM2->CCR4 =0;
			GPIOB->ODR = ~(0b0110); //change direction
			crossing= crossing+1;
		}
		move();					//Move forward according to sensor positions to achieve set position

		//TEMPORARY CODE TEST H-BRIDGE
		if (((GPIOA->IDR)&(0b00001000))==0){
			//GPIOB->ODR = 0b1001;
			}
	}
}
//END OF MAIN
void init_ports(void){
	RCC->AHBENR |= ( RCC_AHBENR_GPIOAEN| RCC_AHBENR_GPIOBEN);
	GPIOB->MODER |= 0x5555;
	/*GPIOB->MODER |= (GPIO_MODER_MODER2_0|GPIO_MODER_MODER3_0|GPIO_MODER_MODER4_0|
	GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0| GPIO_MODER_MODER7_0);*/
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0|GPIO_PUPDR_PUPDR1_0|GPIO_PUPDR_PUPDR2_0 |GPIO_PUPDR_PUPDR3_0);  //
	GPIOA->MODER |= (GPIO_MODER_MODER5|GPIO_MODER_MODER6);     //PA5&PA6 ANALOG,ADCCHNL 5&6
	//GPIOB->MODER |= (GPIO_MODER_MODER0|GPIO_MODER_MODER1);     //PB0&PB1 ANALOG,ADC CHNL 8&9

}

//config PWM 
void init_timer(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOB->MODER |= GPIO_MODER_MODER10_1; // PB10 = AF
    GPIOB->MODER |= GPIO_MODER_MODER11_1; // PB11 = AF
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR10&(GPIO_AF2<<8));
    GPIOB->AFR[1] |= (GPIO_AFRH_AFR11&(GPIO_AF2<<12));
    TIM2->ARR = 48000; // f = 1 KHz
    TIM2->PSC = 0;
     // specify PWM mode: OCxM bits in CCMRx. We want mode 1
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1
    TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1
    // enable the OC channels
    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->CCER |= TIM_CCER_CC4E;
    TIM2->CR1 |= TIM_CR1_CEN; // counter enable
}

//READ SESNOSR VALUES
void ReadLine(){
	//ALL SESNING?
	if((((GPIOA->IDR)&(0b100))==0)&(((GPIOA->IDR)&(0b10))==0)&(((GPIOA->IDR)&(0b1))==0)){
		crossing=crossing+1;
		GPIOB->ODR = 0b100011;
	}
	//All NOT SESNSING
	else if((((GPIOA->IDR)&(0b100))!=0)&(((GPIOA->IDR)&(0b10))!=0)&(((GPIOA->IDR)&(0b1))!=0)){
		GPIOB->ODR = 0b11011;
		if(previous_pos==3){
			position = 4;
		}
		else if(previous_pos==1)
			position = 0;
	}
	//IS PA1 SENSING?
	else if(((GPIOA->IDR)&(0b010))==0){
		position = 2;
	}
	//IS PA2 SESNING?
	else if(((GPIOA->IDR)&(0b100))==0){
		position = 3;
	}
	//IS PA0 SESNING?
	else if(((GPIOA->IDR)&(0b001))==0){
		position = 1;
	}
}

void move(void){
		//display debugger info on screen 
		lcd_command(CLEAR);
		sprintf(collection, "S: %3d", position);
		lcd_putstring(collection);
		lcd_command(LINE_TWO);
		sprintf(collection, "E: %3d", Integral);
		lcd_putstring(collection);
		
		Error = position-2;
		Integral = Integral +Error;
		int adjust = Kp*Error+ Ki*Integral;
		int LeftSpeed = LmotorBaseSpeed+adjust;
		int RightSpeed = RmotorBaseSpeed-adjust;
		//KEEP SPEED WITHIN RANGE
		if(RightSpeed>MaxSpeed)RightSpeed = MaxSpeed;
		if(LeftSpeed>MaxSpeed)LeftSpeed = MaxSpeed;
		if(RightSpeed<4800)RightSpeed = 4800;
		if(LeftSpeed<4800)LeftSpeed = 4800;

		//SET MOTOR SPEEDS ACCORDIGNLY
		TIM2->CCR3 =LeftSpeed; //duty circle = ccr/(arr+1)   TIM2->ARR = 48000; // f = 1 KHz
		TIM2->CCR4 =RightSpeed; // Max CCR = 48 001
		previous_pos = position;
}
//for soft start
void Delay (void)
{
	for( delay1=0; delay1<255; delay1++){
		for( delay2=0; delay2<5; delay2++)
	}
}
//********************************************************************
// END OF PROGRAM
