/*
 * PID.c
 *
 *  Created on: Jul 5, 2022
 *      Author: Karol
 */

#include "PID.h"

void pid_init(pid_data *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init)
{
	pid_data->previous_e = 0;
	pid_data->total_e = 0;

	pid_data->Kp = kp_init;
	pid_data->Ki = ki_init;
	pid_data->Kd = kd_init;

	pid_data->anti_windup_limit = anti_windup_limit_init;
}

void pid_reset(pid_data* pid_data)
{
	pid_data->total_e = 0;
	pid_data->previous_e = 0;
}

int pid_calculate(pid_data* pid_data, int wartosc_zadana, int wartosc_mierzona)
{
	int e;											//uchyb
	float p_temp, i_temp, d_temp;					//człony P, I, D

	e = wartosc_zadana - wartosc_mierzona;			//obliczenie uchybu
	if (e>2000) return 0;
	pid_data->total_e += e;							//sumowanie uchybu

	p_temp = (float)(pid_data->Kp * e);								//odpowiedź członu proporcjonalnego
	i_temp = (float)(pid_data->Ki * pid_data->total_e);				//odpowiedź członu całkującego
	d_temp = (float)(pid_data->Kd * (e - pid_data->previous_e));	//odpowiedź członu różniczkującego

	if(i_temp >= pid_data->anti_windup_limit) i_temp = pid_data->anti_windup_limit;			//Anti-Windup - ograniczenie odpowiedzi członu całkującego
	else if(i_temp <= -pid_data->anti_windup_limit) i_temp = -pid_data->anti_windup_limit;

	pid_data->previous_e = e;					//aktualizacja zmiennej z poprzednią wartością błędu

	return (int)(p_temp + i_temp + d_temp);		//odpowiedź regulatora
}
