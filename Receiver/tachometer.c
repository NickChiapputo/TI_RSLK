#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <tachometer.h>

#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1
#define MOTOR_FORWARD	1
#define MOTOR_STOP		0
#define MOTOR_BACKWARD	-1

volatile int leftTacho_timerCount = 0;
volatile int leftTacho_direction = MOTOR_FORWARD;

volatile int rightTacho_timerCount = 0;
volatile int rightTacho_direction = MOTOR_FORWARD;

int clockSMCLK;

void initTachometers( int clockSMCLK )
{
	clockSMCLK = clockSMCLK;

	Timer_A_ContinuousModeConfig Timer_A_config =
	{
			TIMER_A_CLOCKSOURCE_SMCLK,  //3MHz
			TIMER_A_CLOCKSOURCE_DIVIDER_5,  //600kHz
			TIMER_A_TAIE_INTERRUPT_DISABLE,
			TIMER_A_DO_CLEAR
	};

	Timer_A_CaptureModeConfig leftTacho_captureConfig =
	{
			TIMER_A_CAPTURECOMPARE_REGISTER_1,
			TIMER_A_CAPTUREMODE_RISING_EDGE,
			TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
			TIMER_A_CAPTURE_SYNCHRONOUS,
			TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
			TIMER_A_OUTPUTMODE_OUTBITVALUE
	};


	Timer_A_CaptureModeConfig rightTacho_captureConfig =
	{
			TIMER_A_CAPTURECOMPARE_REGISTER_0,
			TIMER_A_CAPTUREMODE_RISING_EDGE,
			TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
			TIMER_A_CAPTURE_SYNCHRONOUS,
			TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
			TIMER_A_OUTPUTMODE_OUTBITVALUE
	};

	Timer_A_configureContinuousMode(TIMER_A3_BASE, &Timer_A_config);

	//P10.5/TA3.1: left tachometer
	//P10.4/TA3.0: right tachometer
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

	//TA3.1: left tachometer pulse period (i.e., speed) measurement
	//TA3.0: right tachometer pulse period (i.e., speed) measurement
	Timer_A_initCapture(TIMER_A3_BASE, &leftTacho_captureConfig);
	Timer_A_initCapture(TIMER_A3_BASE, &rightTacho_captureConfig);

	//P5.2: left tachometer direction
	//P5.0: right tachometer direction
	GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN2);

	Interrupt_enableInterrupt(INT_TA3_0);
	Interrupt_enableInterrupt(INT_TA3_N);
}

void startTacho()
{
	Timer_A_startCounter( TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE );
}

int getTachoDirection( int motorSel )
{
	return motorSel == LEFT_MOTOR ? leftTacho_direction : rightTacho_direction;
}

float getSpeed( int motorSel )
{
	// speed in RPM = [ 60 / ( 360 * pulsePeriod ) ] * ( clockSMCLK / 5 )
	//				= [ 1 / ( 6 * pulsePeriod ) ] * ( clockSMCLK / 5 )
	//				= clockSMCLK / ( 30 * pulsePeriod )
	//				= 100kHz / pulsePeriod
	return clockSMCLK / ( 30 * ( motorSel == LEFT_MOTOR ? leftTacho_timerCount : rightTacho_timerCount ) );
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
