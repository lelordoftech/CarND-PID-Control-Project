#include "PID.h"
#include <math.h>       /* fabs */

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  pre_error = 0;
}

void PID::UpdateError(double cte, double dt)
{
  p_error = cte;

  i_error += cte * dt;
  if (cte == 0)
  {
    i_error = 0;
  }
  if (fabs(cte) > 70)
  {
    i_error = 0;
  }

  d_error = (cte - pre_error) / dt;

  pre_error = cte;
}

double PID::TotalError()
{
  return Kp_*p_error + Ki_*i_error + Kd_*d_error;
}
