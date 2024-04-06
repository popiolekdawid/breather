/*
 * PID.c
 *
 *  Created on: Jul 5, 2022
 *      Author: Karol
 */

#include "PID.h"

void pid_init(pid_data *pd, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init)
{
	pd->previous_e = 0;
	pd->total_e = 0;

	pd->Kp = kp_init;
	pd->Ki = ki_init;
	pd->Kd = kd_init;

	pd->anti_windup_limit = anti_windup_limit_init;
}

void pid_reset(pid_data* pid_data)
{
	pid_data->total_e = 0;
	pid_data->previous_e = 0;
}

int pid_calculate(pid_data* pd, int wartosc_zadana, int wartosc_mierzona)
{
	int e;											//uchyb
	float p_temp, i_temp, d_temp;					//człony P, I, D

	e = wartosc_zadana - wartosc_mierzona;			//obliczenie uchybu
	if (e>2000) return 0;
	pd->total_e += e;							//sumowanie uchybu

	p_temp = (float)(pd->Kp * e);								//odpowiedź członu proporcjonalnego
	i_temp = (float)(pd->Ki * pd->total_e);				//odpowiedź członu całkującego
	d_temp = (float)(pd->Kd * (e - pd->previous_e));	//odpowiedź członu różniczkującego

	//if(i_temp >= pd->anti_windup_limit) i_temp = pd->anti_windup_limit;			//Anti-Windup - ograniczenie odpowiedzi członu całkującego
	//else if(i_temp <= -pd->anti_windup_limit) i_temp = -pd->anti_windup_limit;

	pd->previous_e = e;					//aktualizacja zmiennej z poprzednią wartością błędu

	return (int)(p_temp + i_temp + d_temp);		//odpowiedź regulatora
}
