#ifndef PID_H
#define PID_H

#include <math.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double last_cte;
  double sum_cte;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  /*
   * Saturation values
   * [-1,1] for steering
   *
   */
  const double sat_upper = 1.0;
  const double sat_lower = -1.0;
  const double Max_ctrl_out = 10;

  /*
   * Saturation flag
   */
  bool is_saturated;
  /*
   * Current calculated output w/o saturation
   */
  double curr_out;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);

  double sat(double, double, double);
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Get the actuator response value
   */
  double Response(void);
};

#endif /* PID_H */
