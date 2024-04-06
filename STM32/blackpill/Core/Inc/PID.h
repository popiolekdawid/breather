/*
 * PID.h
 *
 *  Created on: June 28, 2023
 *      Author: Weronika & Dawid
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	int previous_e; 			//Poprzedni błąd dla członu różniczkującego
	int total_e;				//Suma uchybów dla członu całkującego
	float Kp;					//Wzmocnienie członu proporcjonalnego
	float Ki;					//Wzmocnienie członu całkującego
	float Kd;					//Wzmocnienie członu różniczkującego
	int anti_windup_limit;		//Anti-Windup - ograniczenie członu całkującego
} pid_data;

void pid_init(pid_data*, float, float, float, int anti_windup_limit_init);
void pid_reset(pid_data* pid_data);
int pid_calculate(pid_data*, int wartosc_zadana, int wartosc_mierzona);

#endif /* INC_PID_H_ */
