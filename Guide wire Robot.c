
//  Copyright (c) 01-APR-2017 Samin Islam LOLOLOL
//  ~C51~

#include <C8051F38x.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define SYSCLK    48000000L // SYSCLK frequency in Hz
#define BAUDRATE  115200L   // Baud rate of UART in bps

#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  4
#define TURN_VOLT 0.12
#define INTERSECTION_VOLT 0.3

#define OUT0 P2_4
#define OUT1 P2_5

#define OUT2 P2_7
#define OUT3 P2_6

//#define TEST2 P2_2
//#define TEST P2_1
//#define TEST1 P2_0
#define LCD_RS P1_6
#define LCD_RW P1_5 // Not used in this code
#define LCD_E  P1_4
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16

#define stop P0_2
#define parallel P0_6


volatile unsigned char pwm_count=0;
float wave1;
float wave2;
float wave3;
float wave4;
float v0;
float v1;
float v2;
float v3;
float vd;
float v;
unsigned char j;
int count=0;
char printyprint[CHARS_PER_LINE];

char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	VDM0CN=0x80; // enable VDD monitor
	RSTSRC=0x02|0x04; // Enable reset on missing clock detector and VDD

	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == 12000000L)
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == 24000000L)
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == 48000000L)
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12000000L, 24000000L, or 48000000L
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency

	
	// Configure P2.0 to P2.3 as analog inputs
	P2MDIN &= 0b_1111_0000; // P2.0 to P2.3
	P2SKIP |= 0b_0000_1111; // Skip Crossbar decoding for these pins

	// Init ADC multiplexer to read the voltage between P2.0 and ground.
	// These values will be changed when measuring to get the voltages from
	// other pins.
	// IMPORTANT: check section 6.5 in datasheet.  The constants for
	// each pin are available in "c8051f38x.h" both for the 32 and 48
	// pin packages.
	AMX0P = LQFP32_MUX_P2_0; // Select positive input from P2.0
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN=0b_0000_1000; //Select VDD as the voltage reference for the converter
  	
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	P0MDOUT|=0x10;     // Enable Uart TX as push-pull output
	XBR0=0x01;         // Enable UART on P0.4(TX) and P0.5(RX)
	XBR1=0x40;         // Enable crossbar and weak pull-ups
	
	
	// Configure UART0
	SCON0 = 0x10; 
#if (SYSCLK/BAUDRATE/2L/256L < 1)
	TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
	CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
	CKCON |=  0x08;
#elif (SYSCLK/BAUDRATE/2L/256L < 4)
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01                  
	CKCON |=  0x01;
#elif (SYSCLK/BAUDRATE/2L/256L < 12)
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
#else
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
	CKCON |=  0x02;
#endif
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
	
	// Configure the pins used for square output
	P1MDOUT|=0b_0000_1111;
	P2MDOUT|=0b_0000_0111;
	P0MDOUT |= 0x10; // Enable UTX as push-pull output
	XBR0     = 0x01; // Enable UART on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0x40; // Enable crossbar and weak pull-ups

	// Initialize timer 2 for periodic interrupts
	TMR2CN=0x00;   // Stop Timer2; Clear TF2;
	CKCON|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(2*48))/(100L)); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2

	EA=1; // Enable interrupts
	
	return 0;
}

void ADC_get(void);
// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}



void LCD_pulse (void)
{
	LCD_E=1;
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
	// The accumulator in the C8051Fxxx is bit addressable!
	ACC=x; //Send high nible
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	//Timer3us(40);
	ACC=x; //Send low nible
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	//waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	//waitms(5);
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	LCD_RW=0; // We are only writing to the LCD in this program
	//waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	//waitms(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, bit clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	//waitms(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

void Turn_left(void){
			wave1=10;
			wave2=0;
			wave3=70;
			wave4=0;
			LCDprint("Turning Right", 1, 1);
			LCDprint("RPM = 20.1", 2, 1);
}

void Turn_right(void){
			wave1=70;
			wave2=0;
			wave3=10;
			wave4=0;
			LCDprint("Turning Left", 1, 1);
			LCDprint("RPM = 20.1", 2, 1);
}

void Go_straight(void){
			wave1=70;
			wave2=0;
			wave3=70;
			wave4=0;
			LCDprint("Going Straight", 1, 1);
			LCDprint("RPM = 23", 2, 1);
		
}

void Rotate_180(void){
			wave1=70;
			wave2=0;
			wave3=0;
			wave4=70;
			LCDprint("Rotating", 1, 1);
			LCDprint("RPM = 18", 2, 1);
			waitms(1300);
			
}

void STOP(void) {
			wave1=0;
			wave2=0;
			wave3=0;
			wave4=0;
			LCDprint("Halt", 1, 1);
			LCDprint("RPM: 0", 2, 1);
}

void Turn_right_reverse(void){
			wave1=0;
			wave2=50;
			wave3=0;
			wave4=20;
			LCDprint("Turning Right", 1, 1);
			LCDprint("RPM: 15.6", 2, 1);
		

}

void Turn_right_reverse1(void){
			wave1=0;
			wave2=90;
			wave3=0;
			wave4=30;
			LCDprint("Parallel Parking", 1, 1);
			LCDprint("RPM: 26.4", 2, 1);
		

}

void Turn_left_reverse(void){
			wave1=0;
			wave2=20;
			wave3=0;
			wave4=50;
			LCDprint("Turning Left", 1, 1);
			LCDprint("RPM: 15.6", 2, 1);
	
}
void Turn_left_reverse1(void){
			wave1=0;
			wave2=30;
			wave3=0;
			wave4=90;
			LCDprint("Parallel Parking", 1, 1);
			LCDprint("RPM: 26.8", 2, 1);
	
}

void Go_straight_reverse(void){
			wave1=0;
			wave2=40;
			wave3=0;
			wave4=40;
			LCDprint("Going Reverse", 1, 1);
			LCDprint("RPM: 14.6", 2, 1);
		
}

void Go_straight_reverse1(void){
			wave1=0;
			wave2=100;
			wave3=0;
			wave4=100;
			LCDprint("Parallel Parking", 1, 1);
			LCDprint("RPM: 28.5", 2, 1);
		
}

void Reverse(void){
		if(v0>v2&&vd>TURN_VOLT&&v1<=INTERSECTION_VOLT){
			Turn_right_reverse();
			Go_straight_reverse();
			waitms(50);
		}
		if(v2>v0&&vd>TURN_VOLT&&v1<=INTERSECTION_VOLT){
			Turn_left_reverse();
			Go_straight_reverse();
			waitms(50);
		}
		if((v0==v2||vd<=TURN_VOLT)&&v1<=INTERSECTION_VOLT){
			Go_straight_reverse();
		}	
}

void normal_logic(void){
		//Turning Logic independent of encodings
		if(v0>v2&&vd>TURN_VOLT&&v1<=INTERSECTION_VOLT){
			Turn_left();
		}
		if(v2>v0&&vd>TURN_VOLT&&v1<=INTERSECTION_VOLT){
			Turn_right();
		}
		if((v0==v2||vd<=TURN_VOLT)&&v1<=INTERSECTION_VOLT){
			Go_straight();
		}
}

void count_normal(void){
	while(v3<=0.01&&(v0<=0.01||v2<=0.01)){
			count = 0;
			while(v3<=0.01&&(v0<=0.01||v2<=0.01)){
			printf("\x1B[1;1H");
			ADC_get();
			if(v3<=0.01&&(v0<=0.01||v2<=0.01)){
				normal_logic();
				count++;
			}
			else {
				printf("\x1B[21;1H");
				printf("Counter: %d\r\n", count);
				//break;
			}
		}
		//break;
		}
}


void Parallel_Park(void){
	LCDprint("Parallel Parking", 1, 1);
	Go_straight_reverse1();
	waitms(500);
	Turn_right_reverse1();
	waitms(700);
	Turn_left_reverse1();
	waitms(750);
	LCDprint("Done!", 1, 1);
	wave1 = 0;
	wave2 = 0;
	wave3 = 0;
	wave4 = 0;
	waitms(10000);
}

void ADC_get(void)
{
		for(j=0; j<NUM_INS; j++)
		{
			AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			
			// Select next channel while ADC0 is busy
			switch(j)
			{
				case 0:
					AMX0P=LQFP32_MUX_P2_1;
				break;
				case 1:
					AMX0P=LQFP32_MUX_P2_2;
				break;
				case 2:
					AMX0P=LQFP32_MUX_P2_3;
				break;
				case 3:
					AMX0P=LQFP32_MUX_P2_0;
				break;
			}
			
			while (AD0BUSY); // Wait for conversion to complete
			v = ((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			
			// Display measured values
			switch(j)
			{
				case 0:
					printf("Left P2.0=%5.3fV, ", v);
					v0=v;
				break;
				case 1:
					printf("Intersection P2.1=%5.3fV, ", v);
					v1=v;
				break;
				case 2:
					printf("Right P2.2=%5.3fV, ", v);
					v2=v;
				break;
				case 3:
					printf("Instruction P2.3=%5.3fV", v);
					v3=v;
				break;
			}
		}
		if(parallel==0){
		Parallel_Park();
		}
		waitms(50);  // Wait 10ms before next round of measurements.
}



void Timer2_ISR (void) interrupt 5
{
	TF2H = 0; // Clear Timer2 interrupt flag
	
	pwm_count++;
	if(pwm_count>100) pwm_count=0;
	
	OUT0=pwm_count>wave1?0:1;
	OUT1=pwm_count>wave2?0:1;
	OUT2=pwm_count>wave3?0:1;
	OUT3=pwm_count>wave4?0:1;
	
	/*if(v3<=0.01)
	{
		count++;
	}
	if(count>6) count=0;*/
}


void main (void)
{
	
	int flag=1;
	LCD_4BIT();
	
	while(1){
	flag=1;
	wave1=0;
	wave2=0;
	wave3=0;
	wave4=0;

    
	

	while(1)
	{

		if(stop==0){
			wave1=0;
			wave2=0;
			wave3=0;
			wave4=0;
			flag=0;
		}
		
		vd=v0-v2;
		if(vd<0){
			vd=v2-v0;
		}
	
		printf("\x1B[1;1H"); // ANSI escape sequence: move to row 1, column 1
		count_normal();
		ADC_get();
		printf("\r\n");
		printf("v0 = %5.3f, ", v0);
		printf("v1 = %5.3f, ", v1);
		printf("v2 = %5.3f, ", v2);
		printf("v3 = %5.3f, ", v3);
		printf("vd=%5.3f", vd);
		
		normal_logic();
		
		if((count>=2&&count<=4)&&v1>INTERSECTION_VOLT)
		{
			//Intersection up ahead
			Turn_right();
			waitms(1000);
			count=0;
			printf("\x1B[15;1H");
			printf("in 2, 4");
		}
		if((count>=5&&count<=9)&&v1>INTERSECTION_VOLT)
		{
			//Intersection up ahead
			Turn_left();
			waitms(1100);
			count=0;
			printf("\x1B[16;1H");
			printf("in 6, 9");
		}
		if(count>=10&&count<=14/*&&(v1>INTERSECTION_VOLT||v1<=INTERSECTION_VOLT)*/)
		{
			Rotate_180();
			count=0;	
			printf("\x1B[17;1H");
			printf("in 10, 12");	
	    }
	    if(count>=15&&count<=17){
			Go_straight();
			count = 0;
			printf("\x1B[18;1H");
			printf("in 13, 17");
		}
	    while(count>=18&&count<=21/*&&(v1>INTERSECTION_VOLT||v1<=INTERSECTIN_VOLT)*/)
		{
			printf("\x1B[1;1H");
			ADC_get();
			if(v3<=0.01&&(v0<=0.01||v2<=0.01)) {//Intersection up ahead
				count = 0;
			    break;
			}
			if((count>=18&&count<=21)){
			STOP();
			//waitms(800);
			//count=0;
			printf("\x1B[19;1H");
			printf("in 18, 20");
			}
				
		}
		//if(count>=21&&count<=23)//&&count<=20/*&&(v1>INTERSECTION_VOLT||v1<=INTERSECTION_VOLT)*/)
		
		while(count>=22)
		{	printf("\x1B[1;1H");
			ADC_get();
			vd=v2-v0;
			printf("\x1B[12;1H");
			printf("vd=%5.3f", vd);
			if(vd<0){
				vd=v0-v2;
			printf("\x1B[13;1H");
			printf("vd=%5.3f", vd);
			}
			if(v3<=0.01&&(v0<=0.01||v2<=0.01)) {
				count = 0;
			    //break;
			}
			if(count>=22){
				
			printf("\x1B[14;1H");
			printf("vd=%5.3f", vd);
			if(v3<=0.3){
				Turn_left_reverse();
				printf("\x1B[15;1H");
				printf("Turning left");
				if(v3<=0.25)
					Turn_right_reverse();
				if(v3>0.31)
					Go_straight_reverse();			
			}
			if(v3<=0.3){
				Turn_right_reverse();
				printf("\x1B[15;1H");
				printf("Turning left");
				if(v3<=0.25)
					Turn_left_reverse();
				if(v3>0.31)
					Go_straight_reverse();			
			}
			if(v3>0.3){
				Go_straight_reverse();
				printf("\x1B[15;1H");
				printf("Straight");
			}
				printf("\x1B[20;1H");
				printf("in 21, 23");
			}
		}

					
		
		
		printf("\x1B[4;1H");
		printf("Current Counter: %d\r\n", count);
		printf("Wave 1 of motor 1 has a PWM of: %3.0f \r\n", wave1);
		printf("Wave 2 of motor 1 has a PWM of: %3.0f \r\n", wave2);
		printf("Wave 3 of motor 2 has a PWM of: %3.0f \r\n", wave3);
		printf("Wave 4 of motor 2 has a PWM of: %3.0f \r\n", wave4);

		printf("\r\n");
		
		/*if(ROTATE==0){
			if(wave1<=100){
				wave1++;
			}
		}
		if(DECSPEEDC==0){
			if(wave1>wave2){
				wave1--;
			}
		}
		if(INCSPEEDAC==0){
			if(wave2<=100){
				wave2++;
			}
		}
		if(DECSPEEDAC==0){
			if(wave2>wave1){
				wave2--;
			}
		}*/
		//printf("\x1B[K"); // ANSI escape sequence: Clear to end of line
	}
	}
	}