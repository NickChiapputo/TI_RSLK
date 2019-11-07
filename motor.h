#ifndef MOTOR_H_
#define MOTOR_H_

void initMotors( int );
void increasePWM( int );
void decreasePWM( int );
void switchDirection( int );
void stopMotor( int );
void pauseMotor( int );
void startMotor( int );
void setMotorDutyCycle( int, int );
void setDirection( int, int );
int getMotorDutyCycle( int );
int getMotorDirection( int );

#endif /* MOTOR_H_ */
