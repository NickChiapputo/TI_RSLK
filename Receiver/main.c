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
#define SAMPLE_FREQ		100			// Hz. Frequency to sample controller data

#define HEARTBEAT_FREQ	2  			// unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED 		GPIO_PIN0	// heart beat LED
#define GREEN_LED		GPIO_PIN1
#define BLUE_LED		GPIO_PIN2

#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1
#define MOTOR_FORWARD	1
#define MOTOR_STOP		0
#define MOTOR_BACKWARD	-1

#define SPEED0			15
#define SPEED1			30
#define SPEED2			45
#define SPEED3			60

void initDevice_HFXT();
void initHeartBeatLED();
void initTimer();

uint32_t clockMCLK, clockSMCLK, clockACLK;
uint8_t currentLED = RED_LED;

void main(void)
{
	initDevice_HFXT();
	initHeartBeatLED();
	initTimer();
	initMotors( clockSMCLK );
	initBumpSensors( clockMCLK );
	initTachometers( clockSMCLK );

	Interrupt_enableMaster();
	Timer32_startTimer( TIMER32_0_BASE, false );

	// Start timer for tachometer speed measurements
	startTacho();

	// Start duty cycle monitoring
	Timer32_startTimer( TIMER32_1_BASE, false );

	while( 1 ) {}
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

void initHeartBeatLED()
{
	GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 );
	GPIO_setAsInputPin( GPIO_PORT_P3, GPIO_PIN5 );
	GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 );
	GPIO_setAsInputPin( GPIO_PORT_P1, GPIO_PIN6 );
	GPIO_setAsInputPin( GPIO_PORT_P2, GPIO_PIN3 );
	GPIO_setAsInputPin( GPIO_PORT_P6, GPIO_PIN4 );
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

void T32_INT2_IRQHandler()
{
	Timer32_clearInterruptFlag( TIMER32_1_BASE );

    // Read in control data
	uint8_t enable = GPIO_getInputPinValue( GPIO_PORT_P1, GPIO_PIN6 );
	uint8_t data0 = GPIO_getInputPinValue( GPIO_PORT_P6, GPIO_PIN4 );
	uint8_t data1 = GPIO_getInputPinValue( GPIO_PORT_P3, GPIO_PIN5 );
	uint8_t data2 = GPIO_getInputPinValue( GPIO_PORT_P5, GPIO_PIN1 );
	uint8_t data3 = GPIO_getInputPinValue( GPIO_PORT_P2, GPIO_PIN3 );

	// Check if bump sensors are triggered. Update bump state
	checkBumpState();

	if( enable )
	{
		if( !data0 )
		{
			// Only forward or reverse

			// 00	SPEED0
			// 01	SPEED1
			// 10	SPEED2
			// 11	SPEED3
			uint8_t speed = ( !data2 && !data3 ) ? SPEED0 :
							( !data2 &&  data3 ) ? SPEED1 :
							(  data2 && !data3 ) ? SPEED2 : SPEED3;

			// 0	Forward
			// 1	Reverse
			uint8_t direction = data1 ? MOTOR_BACKWARD : MOTOR_FORWARD;

			// Don't allow robot to move forward when a bump sensor is currently triggered (robot is against a wall)
			if( bumpStateSet() && direction == MOTOR_FORWARD )
				speed = 0;

			setMotorDutyCycle( LEFT_MOTOR,  speed );
			setMotorDutyCycle( RIGHT_MOTOR, speed );

			setMotorDirection( LEFT_MOTOR,  direction );
			setMotorDirection( RIGHT_MOTOR, direction );
		}
		else if( !( data1 && data2 && data3 ) )
		{
			// Turning

			uint8_t speedInside, speedOutside, speedL, speedR, directionL, directionR;

			if( !data2 && !data3 )	// Tight turn, same speed for both motors
			{
				speedInside  = SPEED1;
				speedOutside = SPEED1;
			}
			else if( data2 || data3 )	// Moving turn, higher speed for outside
			{
				speedInside  = SPEED1;
				speedOutside = SPEED2;
			}

			if( data1 )
			{
				// Turn Right
				speedL 		= speedOutside;
				directionL	= data3 ? MOTOR_FORWARD : MOTOR_BACKWARD;

				speedR 		= speedInside;
				directionR	= data2 ? MOTOR_BACKWARD : MOTOR_FORWARD;
			}
			else
			{
				// Turn Left
				speedL		= speedInside;
				directionL	= data2 ? MOTOR_BACKWARD : MOTOR_FORWARD;

				speedR 		= speedOutside;
				directionR	= data3 ? MOTOR_FORWARD : MOTOR_BACKWARD;
			}

			// Don't allow robot to move forward when a bump sensor is currently triggered (robot is against a wall)
			// Can only turn while against a wall if reversing
			if( bumpStateSet() && ( directionL == MOTOR_FORWARD || directionR == MOTOR_FORWARD ) )
			{
				speedL = 0;
				speedR = 0;
			}

			setMotorDutyCycle( LEFT_MOTOR,  speedL );
			setMotorDutyCycle( RIGHT_MOTOR, speedR );

			setMotorDirection( LEFT_MOTOR,  directionL );
			setMotorDirection( RIGHT_MOTOR, directionR );
		}
		else
		{
			// Doing nothing
			setMotorDutyCycle( LEFT_MOTOR,  0 );
			setMotorDutyCycle( RIGHT_MOTOR, 0 );

			setMotorDirection( LEFT_MOTOR,  MOTOR_FORWARD );
			setMotorDirection( RIGHT_MOTOR, MOTOR_FORWARD );
		}
	}
	else
	{
		setMotorDutyCycle( LEFT_MOTOR,  0 );
		setMotorDutyCycle( RIGHT_MOTOR, 0 );

		setMotorDirection( LEFT_MOTOR,  MOTOR_FORWARD );
		setMotorDirection( RIGHT_MOTOR, MOTOR_FORWARD );
	}
}

