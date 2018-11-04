#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

/*
 * Initialize PID parameters
 */
void PID::Init(double Kp_, double Ki_, double Kd_) {
	// initialize errors
	p_error = 0.0;
	d_error = 0.0;
	i_error = 0.0;

	// initialize coefficients
	Kp = Kp_;
	Kd = Kd_;
	Ki = Ki_;
}

/*
 * Update the PID error variables given cross track error
 */
void PID::UpdateError(double cte) {
	double prev_cte = p_error;
	p_error = cte;
	d_error = cte - prev_cte;
	i_error += cte;
}

/*
 * Calculate the total PID error
 */
double PID::TotalError() {
	double total_error = -Kp * p_error -Kd * d_error -Ki * i_error;
	return total_error;
}

