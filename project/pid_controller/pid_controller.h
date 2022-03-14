/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/
#include <vector>

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/
// Twiddle algorithm:
    int index;
    int flag;
    double best_cte;
    std::vector<double> p;
    std::vector<double> dp;
    int twid;

    /*
    * Errors
    */
    double diff_cte;
    double prev_cte;
    double int_cte;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;


    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;
  
    /*
    * Delta time
    */
    double dt;

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
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min, int throti);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


