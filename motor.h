#ifndef MOTOR_H_
#define MOTOR_H_

void initMotors( int );
void increasePWM( int );
void decreasePWM( int );
void switchDirection( int );
void stopMotor( int );
void pauseMotor( int );
void startMotor( int );
static setPWM( int, int );
static setDirection( int, int );
int getPWM( int );
int getMotorDirection( int );

#endif /* MOTOR_H_ */
