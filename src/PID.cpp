#include "PID.h"
#include <math.h>       /* fabs */

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::UpdateError(double cte, double dt)
{
  d_error = (cte - p_error)/dt; // p_error is pre_cte

  i_error += cte*dt;
  if (d_error == 0)
  {
    i_error = 0;
  }
  if (fabs(d_error) > 70)
  {
    i_error = 0;
  }

  p_error = cte;
}

double PID::TotalError()
{
  return - Kp_*p_error - Ki_*i_error - Kd_*d_error;
}
