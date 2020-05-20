#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	prev_cte = 0;
	sum_cte = 0;

	p_error = 0;
	d_error = 0;
	i_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	sum_cte += cte;
	p_error = cte * Kp;
	i_error = sum_cte * Ki;
	d_error = (cte - prev_cte) * Kd;
	prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	return p_error + d_error + i_error;  // TODO: Add your total error calc here!
}