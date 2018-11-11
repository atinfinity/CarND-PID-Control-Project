#include "PID.h"
#include <cfloat>

PID::PID() {
	// initialize twiddle parameters
	step           = 0;
	max_step       = 1500;
	p_iterator     = 0;
	total_iterator = 0;
	total_cte      = 0.0;
	error          = 0.0;
	best_error     = DBL_MAX;
	tolerance      = 0.001;
	sub_move       = 0;
	first          = true;
	second         = true;
}

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

