#include "stm32l476xx.h"
#include <string.h>

//TODO: Display does not communicate with uC on uC power cycle; only when uC halted, display power cycled, uC reset.

//TODO: Drive a GPIO pin high/low at beginning of SysTick and use o-scope to measure period (MOTOR CONTROL IS TIME CRITICAL!!!!!)

#define true 1
#define false 0
#define MAX_MOTOR 40
#define MOTOR_FWD 0x40000000
#define MOTOR_BWD 0x80000000

uint32_t adc_count = 0;
uint8_t sysTickCallCount = 0;
float o2Sensor = 14.7f;
signed int tempSensor = 215;
uint8_t chokePosition = 0;
uint8_t backlight;

char line1[] = "O2 reads .45V (good)";
char line2[] = "Engine temp is: 000F";
char line3[] = " Current choke: 00%";

char line4[20] = "";

uint8_t EoC = false;
uint8_t backlightval;
uint8_t line4clear = true;
												 
void delay(int milliseconds)
{
	int i = 0;
	int endTime = milliseconds * 3150;
	while(i++ < endTime) ;
}

void Configure_Pins()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOEEN;

	GPIOB->MODER &= 0xFFFF0F0F;
	GPIOB->MODER |= 0x0000A000; //Set PB2, 3 to input, PB6, 7 to alternate function
	
	GPIOB->OTYPER |= 0x000000C0; //Set bits 6 and 7 for PB6, 7 to open-drain

	//Set pins PB2 as pull down, PB6, 7 as pull-up	
	GPIOB->PUPDR &= 0xFFFF0F0F;
	GPIOB->PUPDR |= 0x00005020;
	
	GPIOB->AFR[0] |= 0x44000000; // Set PB6, 7 to AF4
	
	// Configure PA1, 2 for ADC inputs
	GPIOA->MODER |= 0x03E; // Set PA0 to output (debug), 1 & 2 to analog
	GPIOA->ASCR |= 0x06; // Connect PA1, 2 to ADC input
	
	GPIOE->MODER &= 0x0FFCFFFF; // Clear PE8, 14, 15
	GPIOE->MODER |= 0x50000000; // Set PE14, 15 to output (8 input)
	GPIOE->ODR &= 0; // Clear ODR for good measures
	
	//Set PE8 to pull down
	GPIOE->PUPDR &= 0xFFFCFFFF;
	GPIOE->PUPDR |= 0x00020000;
	
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_SetPriority(EXTI2_IRQn, 1);
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//Set the external interrupt sources
	SYSCFG->EXTICR[0] &= 0x0;
	SYSCFG->EXTICR[0] |= 0x0100; //PB2 Display
	//interrupt mask register
	EXTI->IMR1 |= EXTI_IMR1_IM2;
	//Enable rising AND falling edge triggers
	EXTI->RTSR1 |= EXTI_RTSR1_RT2;
	EXTI->FTSR1 |= EXTI_FTSR1_FT2;
}

void I2C_Setup()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; //Enable I2C Clock
	
	RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;
	RCC->CCIPR |= RCC_CCIPR_I2C1SEL_1; //Select HSI as I2C clock (16MHz)
	
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
	
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 &= ~I2C_CR1_ANFOFF;
	I2C1->CR1 &= ~I2C_CR1_DNF;
	I2C1->CR1 |= I2C_CR1_ERRIE;
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;
	
	//I2C1->TIMINGR = 0x3 << 28 | 011 << 20 | 0x11 << 16 | 0xBE << 8 | 0xC0; //I2C ~ a little less than 100 kb/s
	I2C1->TIMINGR = 0x3 << 28 | 0x11 << 20 | 0x11 << 16 | 0x17 << 8 | 0x18; //I2C ~ 100 kb/s

	I2C1->OAR1 &= ~I2C_OAR1_OA1EN;
	I2C1->OAR1 = I2C_OAR1_OA1EN | 0x52;
	I2C1->OAR1 &= ~I2C_OAR2_OA2EN;
	
	I2C1->CR2 &= ~I2C_CR2_ADD10;
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 |= I2C_CR2_NACK;
	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start(uint8_t size)
{
	uint32_t tempReg = I2C1->CR2;
	
	tempReg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES |
																			I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
																			I2C_CR2_RD_WRN | I2C_CR2_START |
																			I2C_CR2_STOP));
	
	tempReg &= ~I2C_CR2_RD_WRN; //Write to slave
	
	tempReg |= (uint32_t)(((uint32_t) (0x27 << 1) & I2C_CR2_SADD) | (((uint32_t) size << 16) & I2C_CR2_NBYTES)) | I2C_CR2_START;
	
	I2C1->CR2 = tempReg;

	if( (I2C1->ISR & I2C_ISR_NACKF) == 1 && (GPIOB->IDR & GPIO_IDR_ID2))
		I2C_Start(size);
}

void I2C_Stop()
{
	I2C1->CR2 |= I2C_CR2_STOP;
	
	int watchdog = 0;
	while( (I2C1->ISR & I2C_ISR_STOPF) == 0 && (GPIOB->IDR & GPIO_IDR_ID2) && ++watchdog < 3150); //Wait for stop flag
	
	I2C1->ICR |= I2C_ICR_STOPCF;
}

void I2C_Write(uint8_t data)
{		
	if((GPIOB->IDR & GPIO_IDR_ID2))
	{
		I2C_Start(1);
	
		I2C1->TXDR = data;
			
		int watchdog = 0;
		while( (I2C1->ISR & I2C_ISR_TXE) == 0 && (GPIOB->IDR & GPIO_IDR_ID2) && ++watchdog < 3150);
		
		watchdog = 0;
		//Wait until TC flag set
		while( (I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0 && (GPIOB->IDR & GPIO_IDR_ID2) && ++watchdog < 3150);		
		
		I2C_Stop();
	}
}

void pulseEnable(uint8_t data)
{
	//I2C_Write(data | backlight);
	I2C_Write(data | 0x04 | backlight);	// En high
	delay(2);		// enable pulse must be >450ns
	
	I2C_Write((data | backlight) & ~0x04);	// En low
	delay(5);		// commands need > 37us to settle
}

void write4bit(uint8_t value, uint8_t RW, uint8_t RS)
{
	uint8_t highnib=value&0xf0;
	uint8_t lownib=(value&0x0f) << 4; //Shift left 4 to put data @ D4-D7 (LCD pins)
	
	pulseEnable(highnib | RS | RW);
	pulseEnable(lownib | RS | RW);
	delay(10);
}

void setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > 4 ) {
		row = 3;    // we count rows starting w/0
	}
	write4bit(0x80 | (col + row_offsets[row]), 0x00, 0x00);
}

void printString(char* firstChar)
{
	while(*firstChar != '\0' && (GPIOB->IDR & GPIO_IDR_ID2))
	{
		write4bit(*firstChar, 0x00, 0x01);
		firstChar++;
	}
}

void updateDisplay()
{		
	//Modify line1 bits 10, 11 O2 voltage, 15-18 for rich/lean/good
	//Modify line2 bits 16, 17, 20 for engine temp
	//Modify line3 bits 15, 16, 17, 18 for choke percentage
	
	line1[10] = (int)((o2Sensor - (int)o2Sensor) * 10) + '0';
	line1[11] = ((int)(((o2Sensor - (int)o2Sensor) * 100)) % 10) + '0';
	if(o2Sensor > 0.5f)
		strncpy(&line1[15], "rich", 4);
	else if (o2Sensor < 0.4f)
		strncpy(&line1[15], "lean", 4);
	else
		strncpy(&line1[15], "good", 4);

	setCursor(10, 0);
	printString(&line1[10]);
	
	//Temp update
	if(tempSensor > 0)
	{
		line2[16] = (((int)tempSensor/100) % 10) + '0';
		line2[17] = (((int)tempSensor/10) % 10) + '0';
		line2[18] = ((int)tempSensor % 10) + '0';
	}
	else
	{
		line2[16] = '-';
		line2[17] = (((int)(tempSensor * -1 ) / 10) % 10) + '0';
		line2[18] = ((int)(tempSensor * -1 ) % 10) + '0';
	}
	setCursor(16,1);
	printString(&line2[16]);
	
	//Choke pos update
	if((GPIOE->IDR & GPIO_IDR_ID8) != 0)
	{
		if(chokePosition == 40)
		{
			line3[15] = '1';
			line3[16] = '0';
			line3[17] = '0';
			line3[18] = '%';
		}
		else
		{
			line3[15] = ' ';
			line3[16] = (chokePosition * 10 / 40) + '0';//(int)((chokePosition / 40.0f) * 10) + '0';
			line3[17] = ((chokePosition * 100 / 40) % 10) + '0';//(int)(((chokePosition / 40.0f) * 1000) / 10) + '0';
			line3[18] = '%';
		}
	}
	else
	{
		line3[15] = ' ';
		line3[16] = '?';
		line3[17] = '?';
		line3[18] = '?';
	}
	
	setCursor(15, 2);
	printString(&line3[15]);
	
	//All warning messages have a non-space character at index 6.
	//line4[6] indicates that the warning line is blank.
	if(line4[6] == ' ')
	{
		if(tempSensor >= 260)
			strncpy(line4, "  Engine very hot!  ", 20);
		if(tempSensor <= 125)
			strncpy(line4, "Let engine warm more", 20);
	}
	
	//This condition block prevents constant blank-string writing
	if(line4[6] == ' ')
	{
		if(!line4clear)
		{
			line4clear = true;
			setCursor(0, 3);
			printString(line4);
		}
	}
	else
	{
		setCursor(0, 3);
		printString(line4);
		line4clear = false;
	}
}

void ADC_Setup()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //Step 1 (book pg 498)
	ADC1->CR &= ~ADC_CR_ADEN; //Step 2
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN; //Step 3
	
	ADC123_COMMON->CCR |= ADC_CCR_VREFEN; //Step 4
	ADC123_COMMON->CCR &= 0xFFC3FFFF; //Step 5 
	//1111 1111 1100 0011 
	//1111 1111 1111 1111
	ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
	ADC123_COMMON->CCR |= 0x00010000;//step 6
	ADC123_COMMON->CCR &= ~ADC_CCR_DUAL;//step 7
	
  //ADC Wakeup
	int wait_time = 0;
	
	if((ADC1->CR & ADC_CR_DEEPPWD) == ADC_CR_DEEPPWD)
		ADC1->CR &= ~ADC_CR_DEEPPWD;
	
	ADC1->CR |= ADC_CR_ADVREGEN;	
	//NVIC->ISER[0] |= 1 <<30; // Something with selecting interrupt source
	
	ADC1->IER |= ADC_IER_EOCIE; // Enables End of Conversion interrupt
		
	while(wait_time++ != 1601) ;
	//end wakeup
	ADC1->CFGR &= 0xFFFFFFC7; //Set data to left-aligned and 12-bits
	
	ADC1->SQR1 &= ~ADC_SQR1_L; //This sets it to do one conversion per ADC_start
	ADC1->SQR1 |= 0x01; // 2 conversions per ADC_Start
	
	ADC1->SQR1 &= 0xFFFFF83F; // Clear 1st channel select
	ADC1->SQR1 |= 0x000000180; //step 12 SETS channel 6 to 1st conversion
	
	ADC1->SQR1 &= 0xFFFE0FFF; // Clear 2nd channel select
	ADC1->SQR1 |= 0x00007000; // Set channel 7 to 2nd conversion

	ADC1->DIFSEL &= 0xFFFFFF3F; // Set channel 6, 7 to single-ended
	ADC1->SMPR1 |= 0x00EC0000; // Set channel 6, 7 sample time to 640.5 ADC clock cycles
	
	ADC1->CFGR &= ~ADC_CFGR_CONT; // Set ADC1 to single conversion mode
	ADC1->CFGR &= ~ADC_CFGR_EXTEN; //Sets software trigger
	
	ADC1->CR |= ADC_CR_ADEN; //step 17
	while((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY) ; //step 18
	
	NVIC_EnableIRQ(ADC1_IRQn); //Enable handler
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	
}

void InitializeLCD()
{
	delay(1000); //Expander needs >=40ms after power up before receiving commands
	backlight = 0;
	write4bit(0x00, 0x00, 0x00);
	
	//Weird initialization stuff	
	write4bit(0x03, 0x00, 0x00);
	delay(10);
	
	write4bit(0x03, 0x00, 0x00);
	delay(5);
	
	write4bit(0x03, 0x00, 0x00);
	delay(3);
	//End weird initialization stuff
	
	
	write4bit(0x02, 0x00 ,0x00); //Put it in 4-bit interface. After this, use write4bit	
	delay(5);
	
	write4bit(0xC2, 0x00, 0x00); //0x2 keeps 4-bit, 0xC sets 2 line, 5x10	
	delay(5);
	
	write4bit(0x80, 0x00, 0x00); //this made cursor go down	
	delay(5);
	
	write4bit(0x10, 0x00, 0x00); //Clear display	
	delay(15);

	write4bit(0x60, 0x00, 0x00); //Set entry mode how you want
	delay(5);
	write4bit(0x02, 0x00, 0x00);
	delay(20);
  
	backlight = 0x08;
	
	setCursor(0, 0);
	printString(line1);
	
	setCursor(0, 1);
	printString(line2);
	
	line3[19] = '\0';
	setCursor(0, 2);
	printString(line3);
}

void EXTI2_IRQHandler(void) //Disp_Switch on
{
	if ((EXTI->PR1 & EXTI_PR1_PIF2) != 0)
	{
			I2C_Setup();
			InitializeLCD();
			
		EXTI->PR1 |= EXTI_PR1_PIF2;
	}
}

void ADC1_IRQHandler()
{
	//Vref ~= 3.12V -> 4095/3.12 = 1,312.5 ~= 1,313
	//Dividing adc data register by 1313 gives value in volts
	
	if(adc_count % 2 == 0) //First conversion
		o2Sensor = ADC1->DR / 1313.0f;
	else //Second conversion
	{
		if(tempSensor != 0)
		{
			tempSensor = ADC1->DR / 1313;
			tempSensor *= (-559);
			tempSensor += 1110; //According to trendline produced by Excel
		}
		
		EoC = true;
	}
	
	adc_count++;
}

void SysTick_Handler()
{
	EoC = false;
	ADC1->CR |= ADC_CR_ADSTART;
	while(!EoC); //Wait for ADC to finish both conversions
	
	if(!(GPIOE->IDR & GPIO_IDR_ID8))
	{
		GPIOE->ODR &= ~MOTOR_FWD; //Actuator should be motionless h-bridge is off
		GPIOE->ODR &= ~MOTOR_BWD;
	}
	
	strncpy(line4, "                    ", 20);
	if(o2Sensor < 0.075f)
		strncpy(line4, "   NO O2 READING!   ", 20);
	else if(tempSensor > 460) //Beyond range of the thermistor
		strncpy(line4, "  NO TEMP READING!  ", 20);
	else
	{
		if((GPIOE->IDR & GPIO_IDR_ID8))
		{
			if(tempSensor > 125)
			{
				if(o2Sensor < 0.4f)
				{
					if(chokePosition != MAX_MOTOR) 
					{
						GPIOE->ODR &= ~MOTOR_BWD; //Need to make sure other half of h-bridge is off
						GPIOE->ODR |= MOTOR_FWD; //Set motor forward
						chokePosition++;
					}
					else
					{
						GPIOE->ODR &= ~MOTOR_FWD;
						strncpy(line4, "  Can't enrich AFR  ", 20);
					}
				}
				else if(o2Sensor > 0.5f)
				{
					if(chokePosition != 0)
					{
						GPIOE->ODR &= ~MOTOR_FWD; //Need to make sure other half of h-bridge is off
						GPIOE->ODR |= MOTOR_BWD; //Set motor backward
						chokePosition--;
					}
					else //AFR rich, but choke is fully open
					{
						GPIOE->ODR &= ~MOTOR_BWD;
						strncpy(line4, "   Can't lean AFR   ", 20);
					}
				}
				else
				{
					GPIOE->ODR &= ~MOTOR_FWD; //Actuator should be motionless h-bridge is off
					GPIOE->ODR &= ~MOTOR_BWD;
				}
			}
			else
			{
				if(chokePosition < MAX_MOTOR)
				{
					GPIOE->ODR &= ~MOTOR_BWD; //Need to make sure other half of h-bridge is off
					GPIOE->ODR |= MOTOR_FWD; //Want to set the choke fully closed 
					chokePosition++;
				}
				else
					GPIOE->ODR &= ~MOTOR_FWD; //Stop at full choke
			}
		}
		else
		{
			chokePosition = 0;
		}
	}

	if(++sysTickCallCount == 5 && (GPIOB->IDR & GPIO_IDR_ID2))
	{
		sysTickCallCount = 0;
		updateDisplay();
	}
}

int main()
{
	// Switch system clock to HSI here
	RCC->CR |= RCC_CR_HSION;
	while( (RCC->CR & RCC_CR_HSIRDY) == 0) ; //Wait for HSI to be ready
	RCC->CFGR |= 0x00000001; //Switch system clock to HSI
	RCC->CR &= 0xFFFFFFFE; //Turn MSI off
			
	Configure_Pins();
	ADC_Setup();
	
	ADC1->CR |= ADC_CR_ADSTART;
	while(!EoC);	
	EoC = false;
	
	line1[10] = (int)((o2Sensor - (int)o2Sensor) * 10) + '0';
	line1[11] = ((int)(((o2Sensor - (int)o2Sensor) * 100)) % 10) + '0';
	if(o2Sensor > 0.5f)
		strncpy(&line4[15], "rich", 4);
	else if (o2Sensor < 0.4f)
		strncpy(&line4[15], "lean", 4);
	else
		strncpy(&line4[15], "good", 4);
	
	//Temp update
	if(tempSensor > 0)
	{
		line2[16] = (((int)tempSensor/100) % 10) + '0';
		line2[17] = (((int)tempSensor/10) % 10) + '0';
		line2[18] = ((int)tempSensor % 10) + '0';
	}
	else
	{
		tempSensor *= -1;
		line2[16] = '-';
		line2[17] = (((int)tempSensor/10) % 10) + '0';
		line2[18] = ((int)tempSensor % 10) + '0';
	}
	
	if((GPIOB->IDR & GPIO_IDR_ID2))
	{
		I2C_Setup();
		InitializeLCD();
	}

	SysTick->CTRL &= 0;
	NVIC_SetPriority(SysTick_IRQn, 2); //ADC interrupt needs higher priority than SysTick
	SysTick->VAL = 0;
	SysTick->LOAD = 799999; //Period of 50ms
	SysTick->CTRL |= 0x07;
	
	while(1) {}
}
