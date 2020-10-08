#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>

enum tuning_state {START=0, INCREASE, DECREASE, NEXT};

class PID {
 public:

  double total_err;
  double best_err;
  int iter;
  
  bool twiddle_;
  std::vector<double> dp;
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_init, double Ki_init, double Kd_init);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  double Control_output();
  void Twiddle(double tol);

 private:
  /**
   * PID Errors
   */
  void increase();
  void decrease2();
  double p_error;
  double i_error;
  double d_error;
  tuning_state state;
  const int count_threshold = 10; // wait for few iteration to stablize the controller
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  int pid_index;
  const double limit_max = 1.0;
  const double limit_min = -1.0;

};

#endif  // PID_H