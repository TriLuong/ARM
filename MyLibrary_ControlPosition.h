#ifndef __MYLIBRARY_ControlPosition_
#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"
#include "math.h"

typedef struct MotorTypeStruct
{
	double Kp;
	double Ki;
	double Kd;
	
	volatile uint16_t stateA,stateB,lastStateAB,stateAB;
	volatile uint32_t Pulse,desPulse;

	double pre_e;
	double e;
	
	double PWM;
	
	uint16_t PWM_Channel;
	uint16_t DIR_PIN;
	uint16_t Channel_A;
	uint16_t Channel_B;
}MotorType;

void InitMotor(MotorType *m, double _Kp, double _Ki, double _Kd, uint16_t _Dir, uint16_t _Channel_A, uint16_t _Channel_B, uint16_t _PWM_Channel);
void readEncoder(MotorType *m,uint16_t _GPIO_Pin);
void PID_Controller(MotorType *m, double theta);
void Forward(MotorType *m);
void Reverse(MotorType *m);
void setController(MotorType *m,TIM_HandleTypeDef *htimPWM);

#endif
