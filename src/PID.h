#ifndef PID_H
#define PID_H

class PID {
 public:
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
  void Init(double Kp_, double Ki_, double Kd_);

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

  /**
   * Update parameter with twiddle algo
   */
  void Twiddle();

  /**
   * Calculates the sum of the twiddle coefficients
   * @output The sum
   */
  double getSumDp();

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double K[3];

  /**
   * Twiddle Variables
   */
  double D[3];
  int tunedParamIndex;
  double maxCounter;
  int counter;
  double latLimit;
};

#endif  // PID_H