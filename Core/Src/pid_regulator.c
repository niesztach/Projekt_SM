
#include "pid_regulator.h"

float32_t calculate_discrete_pid(pid_t2* pid, float32_t setpoint, float32_t measured){
	float32_t u=0, P, I, D, error, integral, derivative;

	error = setpoint-measured;

	//proportional part
	P = pid->param.Kp * error;

	//integral part
	integral = pid->previous_integral + (error+pid->previous_error) ; //numerical integrator without anti-windup
	pid->previous_integral = integral;
	I = pid->param.Ki*integral*(pid->param.dt/2.0);

	//derivative part
	derivative = (error - pid->previous_error)/pid->param.dt; //numerical derivative without filter
	pid->previous_error = error;
	D = pid->param.Kd*derivative;

	//sum of all parts
	u = P  + I + D; //without saturation


	return u;
}

