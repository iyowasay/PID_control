#include "PID.h"
#include <iostream>
#include <vector>
#include <numeric>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_init, double Ki_init, double Kd_init) {

  this->Kp = Kp_init;
  this->Ki = Ki_init;
  this->Kd = Kd_init;

  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  // count_threshold = 10;
  total_err = 0.0;
  best_err = 1000000.0;
  iter = 0;

  pid_index = 0;
  state = START;
  dp = {Kp_init/10.0,Ki_init/10.0,Kd_init/10.0};

}

void PID::UpdateError(double cte) {

  d_error = cte-p_error; // since p_error haven't been updated, it represents the previous crosstrack error 
  
  p_error = cte;
  i_error += cte;

  if(iter > count_threshold) total_err += cte*cte;
  iter++;

}

double PID::Control_output()
{
  double out = - Kp*p_error - Ki*i_error - Kd*d_error;
  // normalize the output [-1,1]
  out = out > limit_max ? limit_max: out;
  out = out < limit_min ? limit_min: out;
  return out;
}

double PID::TotalError() 
{
  if(iter > count_threshold) return total_err/(iter-count_threshold);
  else return 0.0;
}

void PID::increase()
{
  if(pid_index==0){
    this->Kp += dp[pid_index];
  }
  else if(pid_index==1){
    this->Ki += dp[pid_index];
  }
  else if(pid_index==2){
    this->Kd += dp[pid_index];
  }
  return;
}

void PID::decrease2()
{
  if(pid_index==0){
    this->Kp -= 2*dp[pid_index];
  }
  else if(pid_index==1){
    this->Ki -= 2*dp[pid_index];
  }
  else if(pid_index==2){
    this->Kd -= 2*dp[pid_index];
  }
  return;
}

void PID::Twiddle(double tol)
{
  // run the robot
  if(std::accumulate(dp.begin(), dp.end(), 0.0) > tol)
  {
    double current_err = TotalError();
    int current_iter = iter;
    total_err = 0.0;
    iter = 0;

    std::cout << "Twiddle..." << std::endl;
    printf("Iteration = %d, Best error = %e", current_iter, best_err);

    if(state==START)
    {
      best_err = current_err;
      increase();
      state = INCREASE;
    }
    else if(state==INCREASE) 
    { 
      if(current_err < best_err)
      {
        // improved
        best_err = current_err;
        pid_index = (pid_index+1)%3;
        dp[pid_index] *= 1.1;
        state = NEXT;
        // p_error = 0;
        // i_error = 0;
        // d_error = 0;
      }
      else
      {
        decrease2();
        state = DECREASE;
      }
    }
    else if(state==DECREASE) 
    {
      if(current_err < best_err)
      {
        // improved
        best_err = current_err;
        pid_index = (pid_index+1)%3;
        dp[pid_index] *= 1.1;
        state = NEXT;
      }
      else
      {
        increase();
        dp[pid_index] *= 0.9;
        state = INCREASE;
      }
    }
    else if(state==NEXT)
    {
      increase();
      state = INCREASE;
    }

  }
  else
  {
    std::cout << "No need to twiddle." << std::endl;
  }

  return;
  
} 


