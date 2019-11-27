#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <motor.h>
#include <stdio.h>

#define PWM_FREQ    1000 //unit: Hz
#define PWM_DUTYCYCLE_MIN    0 //unit: percent
#define PWM_DUTYCYCLE_MAX    100 //unit: percent
#define PWM_DUTYCYCLE_STEP    5 //unit: percent

#define LEFT_MOTOR    0
#define RIGHT_MOTOR    1

#define MOTOR_FORWARD    1
#define MOTOR_STOP    0
#define MOTOR_BACKWARD    -1

int leftMotor_dutyCycle = PWM_DUTYCYCLE_MIN;
int leftMotor_direction = MOTOR_FORWARD;

int rightMotor_dutyCycle = PWM_DUTYCYCLE_MIN;
int rightMotor_direction = MOTOR_FORWARD;

int clockSMCLK;

void initMotors( int clockSMCLK )
{
	clockSMCLK = clockSMCLK;

	Timer_A_PWMConfig leftMotor_pwmConfig =
	{
			TIMER_A_CLOCKSOURCE_SMCLK,
			TIMER_A_CLOCKSOURCE_DIVIDER_3,
			clockSMCLK/(3*PWM_FREQ),
			TIMER_A_CAPTURECOMPARE_REGISTER_4,
			TIMER_A_OUTPUTMODE_RESET_SET,
			clockSMCLK/(3*PWM_FREQ)/100*leftMotor_dutyCycle
	};

	Timer_A_PWMConfig rightMotor_pwmConfig =
	{
			TIMER_A_CLOCKSOURCE_SMCLK,
			TIMER_A_CLOCKSOURCE_DIVIDER_3,
			clockSMCLK/(3*PWM_FREQ),
			TIMER_A_CAPTURECOMPARE_REGISTER_3,
			TIMER_A_OUTPUTMODE_RESET_SET,
			clockSMCLK/(3*PWM_FREQ)/100*rightMotor_dutyCycle
	};

	//P3.7: 1 to activate left motor;
	//P3.6: 1 to activate right motor
	GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7);

	GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
	GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);

	//P5.4: left motor, 0 for forward, 1 for backward;
	//P5.5: right motor, 0 for forward, 1 for backward
	GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4|GPIO_PIN5);

	GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
	GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);

	//P2.7/TA0.4: left motor PWM;
	//P2.6/TA0.3: right motor PWM
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

	Timer_A_generatePWM(TIMER_A0_BASE, &leftMotor_pwmConfig);
	Timer_A_generatePWM(TIMER_A0_BASE, &rightMotor_pwmConfig);
}

void increasePWM( int motorSel )
{
	if( motorSel == LEFT_MOTOR )
	{
		setMotorDutyCycle( motorSel, leftMotor_dutyCycle + PWM_DUTYCYCLE_STEP );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		setMotorDutyCycle( motorSel, rightMotor_dutyCycle + PWM_DUTYCYCLE_STEP );
	}
}

void decreasePWM( int motorSel )
{
	if( motorSel == LEFT_MOTOR )
	{
		setMotorDutyCycle( motorSel, leftMotor_dutyCycle - PWM_DUTYCYCLE_STEP );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		setMotorDutyCycle( motorSel, rightMotor_dutyCycle - PWM_DUTYCYCLE_STEP );
	}
}

void switchDirection( int motorSel )
{
	if( motorSel == LEFT_MOTOR )
	{
		setMotorDirection( motorSel,
						leftMotor_direction == MOTOR_FORWARD ? MOTOR_BACKWARD :
						leftMotor_direction == MOTOR_BACKWARD ? MOTOR_FORWARD : MOTOR_STOP );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		setMotorDirection( motorSel,
						rightMotor_direction == MOTOR_FORWARD ? MOTOR_BACKWARD :
						rightMotor_direction == MOTOR_BACKWARD ? MOTOR_FORWARD : MOTOR_STOP );
	}
}

void stopMotor( int motorSel )
{
	setMotorDutyCycle( motorSel, 0 );
}

void pauseMotor( int motorSel )
{
	if( motorSel == LEFT_MOTOR )
	{
		Timer_A_setCompareValue( TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 0 );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		Timer_A_setCompareValue( TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 0 );
	}
}

void startMotor( int motorSel )
{
	if( motorSel == LEFT_MOTOR )
	{
		Timer_A_setCompareValue( TIMER_A0_BASE,
									TIMER_A_CAPTURECOMPARE_REGISTER_4,
									clockSMCLK / ( 3 * PWM_FREQ ) / 100 * leftMotor_dutyCycle );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		Timer_A_setCompareValue( TIMER_A0_BASE,
									TIMER_A_CAPTURECOMPARE_REGISTER_3,
									clockSMCLK / ( 3 * PWM_FREQ ) / 100 * rightMotor_dutyCycle );
	}
}

void setMotorDutyCycle( int motorSel, int newDutyCycle )
{
	if( motorSel == LEFT_MOTOR )
	{
		leftMotor_dutyCycle = newDutyCycle > PWM_DUTYCYCLE_MAX ? PWM_DUTYCYCLE_MAX :
								newDutyCycle < PWM_DUTYCYCLE_MIN ? PWM_DUTYCYCLE_MIN : newDutyCycle;

		Timer_A_setCompareValue( TIMER_A0_BASE,
									TIMER_A_CAPTURECOMPARE_REGISTER_4,
									clockSMCLK / ( 3 * PWM_FREQ ) / 100 * leftMotor_dutyCycle );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		rightMotor_dutyCycle = newDutyCycle > PWM_DUTYCYCLE_MAX ? PWM_DUTYCYCLE_MAX :
				newDutyCycle < PWM_DUTYCYCLE_MIN ? PWM_DUTYCYCLE_MIN : newDutyCycle;

		Timer_A_setCompareValue( TIMER_A0_BASE,
									TIMER_A_CAPTURECOMPARE_REGISTER_3,
									clockSMCLK / ( 3 * PWM_FREQ ) / 100 * rightMotor_dutyCycle );
	}
}

void setMotorDirection( int motorSel, int direction )
{
	if( motorSel == LEFT_MOTOR )
	{
		leftMotor_direction = direction;

		if( direction == MOTOR_FORWARD )
			GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN4 );
		else
			GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN4 );
	}
	else if( motorSel == RIGHT_MOTOR )
	{
		rightMotor_direction = direction;

		if( direction == MOTOR_FORWARD )
			GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN5 );
		else
			GPIO_setOutputHighOnPin( GPIO_PORT_P5, GPIO_PIN5 );
	}
}

int getMotorDutyCycle( int motorSel )
{
	return motorSel == LEFT_MOTOR ? leftMotor_dutyCycle : rightMotor_dutyCycle;
}

int getMotorDirection( int motorSel )
{
	return motorSel == LEFT_MOTOR ? leftMotor_direction : rightMotor_direction;
}
