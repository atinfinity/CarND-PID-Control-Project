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
	Ki = Ki_;
	Kd = Kd_;
}

void PID::UpdateError(double cte) {
}

/*
 * Calculate the total PID error
 */
double PID::TotalError() {
	double total_error = p_error + d_error + i_error;
	return total_error;
}

