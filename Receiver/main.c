#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <motor.h>
#include <tachometer.h>
#include <bumpSensors.h>
#include <uart.h>

#define CLOCK_HF		48000000 	// Hz
#define CLOCK_LF		32000 		// Hz
#define SAMPLE_FREQ		1000		// Hz. ADC can't take more than 24 MHz per sample. (P6.0 and P6.1 are each 1 sample). 50 Hz per sample is enough I think

#define HEARTBEAT_FREQ	2  			// unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED 		GPIO_PIN0  // heart beat LED
#define GREEN_LED		GPIO_PIN1
#define BLUE_LED		GPIO_PIN2

#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1
#define MOTOR_FORWARD	1
#define MOTOR_STOP		0
#define MOTOR_BACKWARD	-1

void initDevice_HFXT();
void initHeartBeatLED();
void initTimer();
void initADC14();

uint32_t clockMCLK, clockSMCLK, clockACLK;
uint8_t currentLED = RED_LED;
int backingUp = 0;

const char *terminalDisplayText = 	"\r\nTI-RSLK MAX Motor Control Demo\r\n"
									"  C: Change LED Color, S: Change Direction, I: Increase DutyCycle,\r\n"
									"  D: Decrease DutyCycle, V: Display Speed, H: Help\r\n"
									"> ";

void main(void)
{
	char str[ 100 ];
	uint8_t data;

	initDevice_HFXT();
	initHeartBeatLED();
	initUART();
	initTimer();
	initADC14();
	initMotors( clockSMCLK );
	initBumpSensors( clockMCLK );
	initTachometers( clockSMCLK );

	Interrupt_enableMaster();
	Timer32_startTimer( TIMER32_0_BASE, false );

	// Start timer for tachometer speed measurements
	startTacho();

	//Initial display on terminal.
	uart0_transmitStr( terminalDisplayText );

	// Start duty cycle monitoring
	Timer32_startTimer( TIMER32_1_BASE, false );

	while(1)
	{
	    if(UART_getInterruptStatus( EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG ) )
		{
			data = UART_receiveData( EUSCI_A0_BASE);
			UART_clearInterruptFlag( EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG );

			switch(data)
			{
			    case 'H':
				case 'h':
					uart0_transmitStr( terminalDisplayText );
					break;
				case 'V':
				case 'v':
					sprintf( str, "Left RPS = %.1f\r\nLeft Direction = %d\r\nRight RPS = %.1f\r\nRight Direction = %d\r\n\n",
							 getSpeed( LEFT_MOTOR ) / 60, getTachoDirection( LEFT_MOTOR ),
							 getSpeed( RIGHT_MOTOR ) / 60, getTachoDirection( RIGHT_MOTOR ) );
					uart0_transmitStr(str);
					break;
			} //end of switch
		} //end of if

		// True when a bump button is pressed.
		if( bumpStateSet() )
		{
			// Check if bump button is still pressed
			int bumpState = checkBumpState();
			if( bumpState == 0 )
			{
				uart0_transmitStr( "Button released.\r\n\n" );

				// Restart motors
				startMotor( LEFT_MOTOR );
				startMotor( RIGHT_MOTOR );
			}
			else
			{
				// Pause motors until button is released.
				pauseMotor( LEFT_MOTOR );
				pauseMotor( RIGHT_MOTOR );

			}
		}
	} //end of while
}

void initDevice_HFXT()
{
	WDT_A_holdTimer();

	PCM_setPowerState( PCM_AM_DCDC_VCORE1 );
	FlashCtl_setWaitState( FLASH_BANK0, 1 );
	FlashCtl_setWaitState( FLASH_BANK1, 1 );

	FPU_enableModule();
	FPU_enableLazyStacking();

	GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
	CS_setExternalClockSourceFrequency( CLOCK_LF, CLOCK_HF );
	CS_startHFXT(false);

	CS_initClockSignal( CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
	CS_initClockSignal( CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8 );
	CS_initClockSignal( CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16 );

	clockMCLK = CS_getMCLK();
	clockSMCLK = CS_getSMCLK();
}

void initADC14()
{
    ADC14_enableModule();
    ADC14_initModule( ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE );

	//Configure P6.1 as A14 and P6.0 as A15
	GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION );

	// Configure non-repeating multi-sequence memory storage in registers 0 and 1
	ADC14_configureMultiSequenceMode( ADC_MEM0, ADC_MEM3, false );

	// Store P6.1 (A14) reading in MEM0 and P6.0 (A15) reading in MEM1
	ADC14_configureConversionMemory( ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false );
	ADC14_configureConversionMemory( ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false );
    ADC14_configureConversionMemory( ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, false );
    ADC14_configureConversionMemory( ADC_MEM3, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, false );

    //See TechRef Section 20.2.6 for sample timing consideration.
    ADC14_enableSampleTimer( ADC_MANUAL_ITERATION );
    ADC14_setSampleHoldTime( ADC_PULSE_WIDTH_4, ADC_PULSE_WIDTH_4 );

    ADC14_enableConversion();
}

void initHeartBeatLED()
{
	GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 );
//	GPIO_setAsInputPin( GPIO_PORT_P2, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
	GPIO_setAsInputPin( GPIO_PORT_P3, GPIO_PIN5 );
	GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 );
	GPIO_setAsInputPin( GPIO_PORT_P1, GPIO_PIN6 );
	GPIO_setAsInputPin( GPIO_PORT_P2, GPIO_PIN3 );
}

void initTimer()
{
    Timer32_initModule( TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE );
    Timer32_setCount( TIMER32_0_BASE, clockMCLK / HEARTBEAT_FREQ );
    Timer32_enableInterrupt( TIMER32_0_BASE );
    Interrupt_enableInterrupt( INT_T32_INT1 );

    Timer32_initModule( TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE );
    Timer32_setCount( TIMER32_1_BASE, clockMCLK / SAMPLE_FREQ );
    Timer32_enableInterrupt( TIMER32_1_BASE );
    Interrupt_enableInterrupt( INT_T32_INT2 );
}

void T32_INT1_IRQHandler()
{
	Timer32_clearInterruptFlag( TIMER32_0_BASE );

	if(GPIO_getInputPinValue( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED ) )
		GPIO_setOutputLowOnPin( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED );
	else
		GPIO_setOutputHighOnPin( GPIO_PORT_P2, currentLED );
}

int roundToNearestN( int x, int N )
{
    return x + ( x % N > ( N / 2 ) ? ( N - x % N ) : ( -1 * x % N ) );
}

void T32_INT2_IRQHandler()
{
	Timer32_clearInterruptFlag( TIMER32_1_BASE );

    // Read in control data
	uint8_t enable = GPIO_getInputPinValue( GPIO_PORT_P1, GPIO_PIN6 );
	uint8_t data0 = GPIO_getInputPinValue( GPIO_PORT_P5, GPIO_PIN2 );
	uint8_t data1 = GPIO_getInputPinValue( GPIO_PORT_P3, GPIO_PIN5 );
	uint8_t data2 = GPIO_getInputPinValue( GPIO_PORT_P5, GPIO_PIN1 );
	uint8_t data3 = GPIO_getInputPinValue( GPIO_PORT_P2, GPIO_PIN3 );

	char str[ 100 ];
	sprintf( str, "%i %i %i %i %i         \r", enable, data0, data1, data2, data3 );
	printf( "%i %i %i %i %i         \r", enable, data0, data1, data2, data3 );

	uint8_t speed = 25;

	if( enable )
	{
		if( data1 )
		{
			setMotorDutyCycle( LEFT_MOTOR,  speed );
			setMotorDutyCycle( RIGHT_MOTOR, speed );

			setMotorDirection( LEFT_MOTOR,  MOTOR_FORWARD );
			setMotorDirection( RIGHT_MOTOR, MOTOR_FORWARD );
		}

		if( data2 )
		{
			setMotorDutyCycle( LEFT_MOTOR,  speed );
			setMotorDutyCycle( RIGHT_MOTOR, speed );

			setMotorDirection( LEFT_MOTOR,  MOTOR_FORWARD  );
			setMotorDirection( RIGHT_MOTOR, MOTOR_BACKWARD );
		}

		if( data3 )
		{
			setMotorDutyCycle( LEFT_MOTOR,  speed );
			setMotorDutyCycle( RIGHT_MOTOR, speed );

			setMotorDirection( LEFT_MOTOR,  MOTOR_BACKWARD );
			setMotorDirection( RIGHT_MOTOR, MOTOR_FORWARD  );
		}

		if( !data1 && !data2 && !data3 )
		{
			setMotorDutyCycle( LEFT_MOTOR,  0 );
			setMotorDutyCycle( RIGHT_MOTOR, 0 );

			setMotorDirection( LEFT_MOTOR,  MOTOR_FORWARD );
			setMotorDirection( RIGHT_MOTOR, MOTOR_FORWARD );
		}
	}
}

