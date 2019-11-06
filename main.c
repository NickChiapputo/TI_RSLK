#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <motor.h>
#include <tachometer.h>


#define CLOCK_HF    48000000 //48MHz
#define CLOCK_LF    32000 //32kHz

#define HEARTBEAT_FREQ    2  //unit: Hz, heart beat LED blinking frequency for debugging
#define SYSTICK_FREQ    		200			// Hz

#define RED_LED 	GPIO_PIN0  //heart beat LED
#define GREEN_LED	GPIO_PIN1
#define BLUE_LED	GPIO_PIN2

#define BUMP0    GPIO_PIN0  //right side of robot
#define BUMP1    GPIO_PIN2
#define BUMP2    GPIO_PIN3
#define BUMP3    GPIO_PIN5
#define BUMP4    GPIO_PIN6
#define BUMP5    GPIO_PIN7  //left side of robot
#define NUM_DEBOUNCE_CHECKS 10	// For 50 ms debounce time.

#define LEFT_MOTOR    0
#define RIGHT_MOTOR    1

#define MOTOR_FORWARD    1
#define MOTOR_STOP    0
#define MOTOR_BACKWARD    -1

#define NUM_DISP_TEXT_LINE    5
#define MAX_STR_BUFFER_LEN    200

//Function prototypes
void initDevice_HFXT();
void initHeartBeatLED();
void initDebugUART();
void initTimer();
void initBumpSensors();
void initTachometers();

float getSpeed( int );

void uart0_transmitStr(const char *str);


//Global variables
uint32_t clockMCLK, clockSMCLK;
uint8_t currentLED = RED_LED;

volatile uint_fast16_t bumpStatus = 0;
volatile uint_fast16_t bumpPress = 0;
volatile uint32_t bumpButton;
volatile uint32_t buttonStateIndex;

volatile int leftTacho_timerCount = 0;
volatile int leftTacho_direction = MOTOR_FORWARD;

volatile int rightTacho_timerCount = 0;
volatile int rightTacho_direction = MOTOR_FORWARD;

char strBuffer[MAX_STR_BUFFER_LEN];

const char *terminalDisplayText[NUM_DISP_TEXT_LINE] =
{
	"\r\n",
	"TI-RSLK MAX Motor Control Demo\r\n",
	"  C: Change LED Color, L: Left Motor, R: Right Motor, S: Change Direction\r\n",
	"  I: Increase DutyCycle, D: Decrease DutyCycle, V: Display Speed, H: Help\r\n",
	"> "
};


void main(void)
{
	int i;
	char str[ 100 ];
	uint8_t data;

	initDevice_HFXT();
	initHeartBeatLED();
	initDebugUART();
	initTimer();
	initMotors( clockSMCLK );
	initBumpSensors();
	initTachometers();

	Interrupt_enableMaster();
	Timer32_startTimer(TIMER32_0_BASE, false);

	// Start timer for tachometer speed measurements
	Timer_A_startCounter( TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE );

	//Initial display on terminal.
	for(i=0; i<NUM_DISP_TEXT_LINE; i++)
	{
		uart0_transmitStr(terminalDisplayText[i]);
	}

	while(1)
	{
		if(UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG))
		{
			data = UART_receiveData(EUSCI_A0_BASE);
			UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

			switch(data)
			{
				case 'C':
				case 'c':
					currentLED = (currentLED == BLUE_LED) ? RED_LED : (currentLED << 1); //cycle through three colors
					uart0_transmitStr("Change blinking LED color.\r\n> ");
					break;

				case 'H':
				case 'h':
					for(i=0; i<NUM_DISP_TEXT_LINE; i++)
					{
						uart0_transmitStr(terminalDisplayText[i]);
					}
					break;

				case 'S':
				case 's':
					switchDirection( LEFT_MOTOR );
					switchDirection( RIGHT_MOTOR );

					sprintf( str, "Change motor direction: %i, %i.\r\n> ", getDirection( LEFT_MOTOR ), getDirection( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;

				case 'I':
				case 'i':
					increasePWM( LEFT_MOTOR );
					increasePWM( RIGHT_MOTOR );

					sprintf( str, "Increase PWM duty cycle: %i, %i.\r\n> ", getPWM( LEFT_MOTOR ), getPWM( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;

				case 'D':
				case 'd':
					decreasePWM( LEFT_MOTOR );
					decreasePWM( RIGHT_MOTOR );

					sprintf( str, "Decrease PWM duty cycle: %i, %i.\r\n> ", getPWM( LEFT_MOTOR ), getPWM( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;
				case 'V':
				case 'v':
					sprintf( str, "Left RPS = %.1f\r\nLeft Direction = %d\r\nRight RPS = %.1f\r\nRight Direction = %d\r\n\n",
							 getSpeed( leftTacho_timerCount ) / 60, leftTacho_direction,
							 getSpeed( rightTacho_timerCount ) / 60, rightTacho_direction);
					uart0_transmitStr(str);
					break;
			} //end of switch
		} //end of if

		// Calls when a bump button is pressed.
		if( bumpPress )
		{
			// Check if bump button is still pressed.
			uint8_t buttonState = GPIO_getInputPinValue( GPIO_PORT_P4, bumpButton );
			if( buttonState )
			{
				bumpPress = 0;
				uart0_transmitStr( "Button released.\r\n\n" );

				// Restart motors
				startMotor( LEFT_MOTOR );
				startMotor( RIGHT_MOTOR );
			}
			else
			{
				// Stop motors while bump sensor is active
				pauseMotor( LEFT_MOTOR );
				pauseMotor( RIGHT_MOTOR );
			}
		}
	} //end of while
}

void initDevice_HFXT(void)
{
	WDT_A_holdTimer();  //Stop watchdog timer.

	//Change VCORE to 1 to support a frequency higher than 24MHz.
	//See MSP432 Data Sheet, Section 5.8 for Flash wait-state requirements for active frequency.
	PCM_setPowerState(PCM_AM_DCDC_VCORE1);
	FlashCtl_setWaitState(FLASH_BANK0, 1);
	FlashCtl_setWaitState(FLASH_BANK1, 1);

	FPU_enableModule();
	FPU_enableLazyStacking(); //Required to use FPU within ISR.

	//Configure PJ.2 and PJ.3 in HFXT mode.
	//Initialize external clock sources HFXT.
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	CS_setExternalClockSourceFrequency(CLOCK_LF, CLOCK_HF);
	CS_startHFXT(false);

	CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
	CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8);
	CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);

	clockMCLK = CS_getMCLK();
	clockSMCLK = CS_getSMCLK();
}

void initHeartBeatLED(void)
{
	//Configure P2.0, P2.1, P2.2 as output.
	//P2.0, P2.1, P2.2 are connected to a RGB tri-color LED on LaunchPad.
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);

	Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
	Timer32_setCount(TIMER32_0_BASE, clockMCLK/HEARTBEAT_FREQ);
	Timer32_enableInterrupt(TIMER32_0_BASE);
	Interrupt_enableInterrupt(INT_T32_INT1); // Enable Timer32_0 interrupt in the interrupt controller.
}

void initDebugUART(void)
{
	// Configuration for 3MHz SMCLK, 9600 baud rate.
	// Calculated using the online calculator that TI provides at:
	// http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	const eUSCI_UART_Config config =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK, //SMCLK Clock Source
		19, //BRDIV = 19
		8, //UCxBRF = 8
		0, //UCxBRS = 0
		EUSCI_A_UART_NO_PARITY, //No Parity
		EUSCI_A_UART_LSB_FIRST, //MSB First
		EUSCI_A_UART_ONE_STOP_BIT, //One stop bit
		EUSCI_A_UART_MODE, //UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION //Oversampling
	};

	//Configure GPIO pins for UART. RX: P1.2, TX:P1.3.
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

	UART_initModule(EUSCI_A0_BASE, &config);
	UART_enableModule(EUSCI_A0_BASE);
}

void initTimer()
{
	// Set the SysTick period so that debouncing for the bump sensors can be executed
	SysTick_setPeriod( clockMCLK / SYSTICK_FREQ - 1 );
}

float getSpeed( int timerCount )
{
	//pulsePeriod = timerCount/timerClockFreq
	//speed (RPM) = 60/(360*pulsePeriod)
	//Hard-coded for Timer_A frequency 3MHz/5 = 600kHz.

	// speed in RPM = [ 60 / ( 360 * pulsePeriod ) ] * 600kHz
	//				= [ 1 / ( 6 * pulsePeriod ) ] * 600kHz
	//				= 600kHz / ( 6 * pulsePeriod )
	//				= 100kHz / pulsePeriod
	return 100000.0 / timerCount;
}

void initBumpSensors()
{
	GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P4, BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5 );

	// Set any of the bump buttons to trigger interrupt on falling edge (when pressed)
	GPIO_enableInterrupt( GPIO_PORT_P4, BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5 );
	GPIO_interruptEdgeSelect( GPIO_PORT_P4, BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5, GPIO_HIGH_TO_LOW_TRANSITION );

	Interrupt_enableInterrupt( INT_PORT4 );
}

//Transmit a string through UART0.
void uart0_transmitStr(const char *str)
{
	int len, i=0;

	len = strlen(str);
	while(i < len)
	{
		UART_transmitData(EUSCI_A0_BASE, str[i++]);
		while(!UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG));
		UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG);
	}
}

//Timer32_0 ISR
void T32_INT1_IRQHandler(void)
{
	Timer32_clearInterruptFlag(TIMER32_0_BASE);

	if(GPIO_getInputPinValue(GPIO_PORT_P2, RED_LED|GREEN_LED|BLUE_LED))
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P2, RED_LED|GREEN_LED|BLUE_LED);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P2, currentLED);
	}
}

void PORT4_IRQHandler()
{
	bumpStatus = GPIO_getEnabledInterruptStatus( GPIO_PORT_P4 );
	GPIO_clearInterruptFlag( GPIO_PORT_P4, bumpStatus );

	// Determine which button is pressed
	bumpButton = ( bumpStatus & BUMP0 ) ? BUMP0 :
				( bumpStatus & BUMP1 ) ? BUMP1 :
				( bumpStatus & BUMP2 ) ? BUMP2 :
				( bumpStatus & BUMP3 ) ? BUMP3 :
				( bumpStatus & BUMP4 ) ? BUMP4 : BUMP5;

	// Start SysTick for debouncing the button press(es)
	buttonStateIndex = 0;
	SysTick->VAL = 0;
	SysTick_enableModule();
	SysTick_enableInterrupt();
}

void SysTick_Handler()
{
	// Since buttons are configured with pull-up resistor, the pin value is 0 when pressed
	uint8_t buttonState = GPIO_getInputPinValue( GPIO_PORT_P4, bumpButton );
	buttonState = bumpButton & ( ~buttonState );
	buttonStateIndex++;

	if( ( buttonState & bumpButton ) == 0 )
	{
		SysTick_disableInterrupt();
		SysTick_disableModule();
	}
	else if( buttonStateIndex >= NUM_DEBOUNCE_CHECKS )
	{
		// Button is confirmed pressed
		SysTick_disableInterrupt();
		SysTick_disableModule();

		bumpPress = 1;

		char str[ 100 ];
		sprintf( str, "Bump triggered: %s\r\n", ( bumpStatus & BUMP0 ) ? "BUMP0" :
				( bumpStatus & BUMP1 ) ? "BUMP1" :
				( bumpStatus & BUMP2 ) ? "BUMP2" :
				( bumpStatus & BUMP3 ) ? "BUMP3" :
				( bumpStatus & BUMP4 ) ? "BUMP4" : "BUMP5" );
		uart0_transmitStr( str );
	}
}

//Left tachometer pulse period measurement
void TA3_N_IRQHandler(void)
{
	static uint_fast16_t lastCount = 0, currentCount = 0;

	Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

	currentCount = Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

	leftTacho_timerCount = currentCount - lastCount;
	if(leftTacho_timerCount < 0)
	{
		leftTacho_timerCount += 0xFFFF;
	}

	lastCount = currentCount;

	//P5.2: 1 for forward, 0 for backward
	if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2))
	{
		leftTacho_direction = MOTOR_FORWARD;
	}
	else
	{
		leftTacho_direction = MOTOR_BACKWARD;
	}
}


//Right tachometer pulse period measurement
void TA3_0_IRQHandler(void)
{
	static uint_fast16_t lastCount = 0, currentCount = 0;

	Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	currentCount = Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	rightTacho_timerCount = currentCount - lastCount;
	if(rightTacho_timerCount < 0)
	{
		rightTacho_timerCount += 0xFFFF;
	}

	lastCount = currentCount;

	//P5.0: 1 for forward, 0 for backward
	if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0))
	{
		rightTacho_direction = MOTOR_FORWARD;
	}
	else
	{
		rightTacho_direction = MOTOR_BACKWARD;
	}
}
