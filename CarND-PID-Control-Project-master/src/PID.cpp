#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double iKp, double iKi, double iKd) {
	Kp = iKp;
	Ki = iKi;
	Kd = iKd;
	p_error = 0;
	i_error = 0;
	t_error = 0;
}

void PID::UpdateError(double cte) {
	p_error = cte;
	d_error = cte - p_error;
	i_error += cte;
	t_error += (cte*cte);
}

double PID::Control() {
	double c = -(Kp*p_error) - (Ki*i_error) - (Kd*d_error);
	return c;
}

double PID::TotalError() {
	return t_error;
}

void PID::Twiddle() {
}

