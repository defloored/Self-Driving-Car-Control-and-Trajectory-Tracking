/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <limits>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini, int twidi) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;

    output_lim_max = output_lim_maxi;
    output_lim_min = output_lim_mini;
    diff_cte = 0.0;
    prev_cte = 0.0;
    int_cte = 0.0;

// Twiddle algorithm:
    twid = twidi;
    p = {Kpi, Kii, Kdi};
    dp = {0.001, 0.001, 0.001};
    index = 0;
    flag = 0;
    best_cte = std::numeric_limits<double>::infinity();
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

// Twiddle algorithm:
    if (twid == 1 && ((dp.at(0) + dp.at(1) + dp.at(2)) > 0.0002)) {
        if (flag == 0) {
            int _cte = cte * cte;
            p.at(index) += dp.at(index);
            if (_cte < best_cte) {
                best_cte = _cte;
                dp.at(index) *= 1.1;
                index+=1;
                if (index > 2) {
                    index = 0;
                }
            } else {
                p.at(index) -= 2*dp.at(index);
                flag = 1;
            }
        } else if (flag == 1) {
            int _cte = cte * cte;
            if (_cte < best_cte) {
                best_cte = _cte;
                dp.at(index) *= 1.1;
            } else {
                p.at(index) += dp.at(index);
                dp.at(index) *= 0.9;
            }
            index+=1;
            if (index > 2) {
                index = 0;
            }
            flag = 0;
        }
        Kp = p.at(0);
        Ki = p.at(1);
        Kd = p.at(2);
        cout << "Kp: " << Kp << endl;
        cout << "Ki: " << Ki << endl;
        cout << "Kd: " << Kd << endl;
        cout << "Sol: " << dp.at(0) + dp.at(1) + dp.at(2) << endl << endl;
    }


    if (dt > 0) {
        diff_cte = (cte - prev_cte) / dt;
    } else {
        diff_cte = 0.0;
    }
    prev_cte = cte;
    int_cte += (cte * dt);
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    cout << "dt: " << dt << endl;
    cout << "prev_cte: " << prev_cte << endl;
    cout << "int_cte: " << int_cte << endl;
    cout << "diff_cte: " << diff_cte << endl;
    control = Kp*prev_cte + Kd*diff_cte + Ki*int_cte;
    cout << "control: " << control << endl;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    dt = new_delta_time;
    return dt;
}