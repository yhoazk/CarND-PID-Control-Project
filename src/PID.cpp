#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
 * TODO: with a pid controller increment the speed in proporionally to the steering
*/

PID::PID() {}

PID::~PID() {}

/**
 * This function saturates the output value from the PID controller
 * to the values accepted by the actuator, in this case the steering
 * wheel.
 *  1.0 :: +25°
 * -1.0 :: -25°
 */

double PID::sat(double p, double d, double i)
{
  double  ret_val =0;
  double pp = -Kp * p;
  cout << "Kp: " << pp;
  double dp = -Kd * d;
  cout << " Kd: " << dp;
  double ip = -Ki * i;
  cout << " Ki: " << ip;

  if(is_saturated )
  {
    /* Ignore the integral term */
   ret_val= pp+dp;
  } else{
    ret_val = pp + dp + ip;
  }
  cout << " Pre sat: " << ret_val << endl;
  // Sigmoid saturation
  ret_val = ((2.0f/(1.0f + exp(-0.45 * ret_val))) -1.0f);

  return ret_val;
}

void PID::update(double p, double i, double d){
  Kp = p;
  Ki = i;
  Kd = d;
}

void PID::Init(double Kp_, double Ki_, double Kd_, double up_bound, double low_bound)
{
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0;
  d_error = 0;
  i_error = 0;
  is_saturated = false;
  last_cte = 0;
  sat_upper = up_bound;
  sat_lower = low_bound;
}

/* How is measured the CTE? */
/* By testing when the car is  completely out of the road to the left
 * CTE is -6.0
 * And completely out of the road to the right
 * CTE is +6.0 (actually 5.5)
 * */

void PID::UpdateError(double cte)
{
  p_error = cte;
  d_error = last_cte - cte;
  last_cte = cte;
  i_error += cte;
}

double PID::TotalError()
{

  std::cout << " KI_e: " << i_error << endl;
  std::cout << "KP_e: " << p_error;
  std::cout << " KD_e: " << d_error;
  return 0;
}

double PID::Response()
{
  double t;
  curr_out = fabs(i_error);
  if (curr_out > Max_ctrl_out)
  {
    is_saturated = true;
  } else {
    is_saturated = false;
  }
  t = sat(p_error, d_error, i_error);
  std::cout << "Ctrl Out: " << curr_out << " sat_out: " << t << " is saturated: " << is_saturated << endl;
  return t;
}
