#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <tachometer.h>

void initTachometers()
{
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