/*
 * encoder.c
 *
 *  Created on: Jan 28, 2021
 *      Author: piotr
 */

#include "main.h"

#define ENCODER_RESOLUTION			3
#define TIMER_CONF_BOTH_EDGE_T1T2	4
#define MOTOR_GEAR					150

#define	TIMER_FREQENCY				20
#define	SECOND_IN_MINUTE			60

int speed_table[] = {-1,1,5,- 5};
int im = 0;


void motor_str_init(motor_str *m, TIM_HandleTypeDef *tim)
{
	m->timer = tim;
	m->resolution = ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGE_T1T2 * MOTOR_GEAR;

	m->pulse_count = 0;
	m->measured_speed = 0;
	m->set_speed = 0;
	m->actual_PWM = 0;
}

void motor_calculate_speed(motor_str *m)
{


	//m->measured_speed = speed_table[im++];

	if(im >= 4)
		im = 0;
//	m->measured_speed = (m->pulse_count * TIMER_FREQENCY * SECOND_IN_MINUTE) / m->resolution;

	int output = pid_calculate(&(m->pid_controller), m->set_speed, m->measured_speed);

	m->actual_PWM += output;

	//return m->actual_PWM;
}

void motor_update_count(motor_str *m)
{
	m->pulse_count = (int16_t)__HAL_TIM_GET_COUNTER(m->timer);
	__HAL_TIM_SET_COUNTER(m->timer, 0);
}

void motor_set_speed(motor_str *m, int set_speed)
{
	if(set_speed != m->set_speed)
		pid_reset(&(m->pid_controller));

	m->set_speed = set_speed;
	if(m->set_speed>m->measured_speed)
		m->measured_speed = set_speed+1;
	else if(m->set_speed<m->measured_speed)
		m->measured_speed = set_speed-1;
	else
		m->measured_speed = set_speed-1;
}

void motor_set_measured_speed(motor_str *m, int measured_speed)
{
	if(measured_speed != m->measured_speed)
		pid_reset(&(m->pid_controller));

	m->measured_speed = measured_speed;
}
