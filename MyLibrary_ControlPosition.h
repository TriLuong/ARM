#include "stdio.h"
#include "MyLibrary_ControlPosition.h"


volatile static int8_t QEM[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; //Resolution X4
static const double T=0.01; //Sample time
static const uint16_t PWM_Period=3000;

void InitMotor(MotorType *m, double _Kp, double _Ki, double _Kd, uint16_t _Dir, uint16_t _Channel_A, uint16_t _Channel_B, uint16_t _PWM_Channel)
{
	m->Kp=_Kp;
	m->Ki=_Ki;
	m->Kd=_Kd;
	
	m->DIR_PIN=_Dir;
	m->Channel_A=_Channel_A;
	m->Channel_B=_Channel_B;
	m->PWM_Channel=_PWM_Channel;
}

void readEncoder(MotorType *m,uint16_t _GPIO_Pin)
{
	if(_GPIO_Pin==m->Channel_A || _GPIO_Pin==m->Channel_B)
	{
		m->stateA=HAL_GPIO_ReadPin(GPIOA,m->Channel_A);
		m->stateB=HAL_GPIO_ReadPin(GPIOA,m->Channel_B);
		
		m->stateAB=m->stateA*2+m->stateB;
		uint16_t k=m->lastStateAB*4+m->stateAB;
		
		m->Pulse+=QEM[k];
		m->lastStateAB=m->stateAB;
		
	}
}

void PID_Controller(MotorType *m, double theta)
{
	m->desPulse=(uint32_t)theta*3072.0/360.0;
	m->e=(double)(m->desPulse)-(double)m->Pulse;
	
	double P = m->Kp * m->e;
	double I;
	I+= m->Ki*(m->e)*T;
	double D = m->Kd * (m->e - m->pre_e)/T;
	
	m->PWM=P+I+D;
	
	if(m->PWM > PWM_Period)
		m->PWM=PWM_Period;
	
	
	m->pre_e=m->e;
}

void Forward(MotorType *m)
{
	HAL_GPIO_WritePin(GPIOE,m->DIR_PIN,GPIO_PIN_RESET);
}
void Reverse(MotorType *m)
{
	HAL_GPIO_WritePin(GPIOE,m->DIR_PIN,GPIO_PIN_SET);
}

void setController(MotorType *m,TIM_HandleTypeDef *htimPWM)
{
	if(m->PWM>=0)
		Forward(m);
	else
		Reverse(m);
	__HAL_TIM_SetCompare(htimPWM,m->PWM_Channel,fabs(m->PWM));
}


