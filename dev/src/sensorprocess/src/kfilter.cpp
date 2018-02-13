/*
    Kalman Filter:
    "the problem is that the accelerometer is in general very noisy, and the gyro drifts over time.
       you can only trust the gyroscope on a short term and trust the accelerometer on a long term."
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.4.26  init version - ref: github.com/TKJElectonics/KalmanFilter
    ----
    output: filtered sensor data
    input: sensor data
*/

#include <iostream>

#include "kfilter.h"

using namespace std;

Kfilter::Kfilter(double cfg_Q_angle, double cfg_Q_bias, double cfg_R_measure):
                Q_angle(cfg_Q_angle), Q_bias(cfg_Q_bias), R_measure(cfg_R_measure)
{
    angle = 0;   // reset angle
    bias  = 0;   // reset bias

    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;

    cout << "<Kfilter> instance created." << endl;
}


Kfilter::~Kfilter()
{
}


double Kfilter::getAngle(double newAngle, double newRate, double dt)
{

    /*Step 1: */
    rate = newRate - bias;
    angle += dt * rate;

    /*Step 2: update estimation error covariance-Project the error covariance ahead*/
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    /*Step 4: Measurement update, Calculate Kalman gain */
    S = P[0][0] + R_measure;    // Estimate error
    //cout << "<Kfilter> S:" << S  << ". R_measure:" << R_measure << ". Q_bias:" << Q_bias <<". P[0][0]:" << P[0][0] << endl;

    /*Step 5: */
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    /*Step 3: Update estimate with measurement zk(new angle)*/
    //printf ("<Kfilter>: angle: %f, newAngle:%f, S: %f, K[0]:%f, K[1]:%f  \n", angle, newAngle, S, K[0], K[1]);
    y = newAngle - angle;

    printf ("<Kfilter>: angle: %f, y:%f, K[0]:%f, bias: %f \n", angle, y, K[0], bias);

    /*Step 6*/
    angle += K[0] * y;
    bias  += K[1] * y;

    /*Step 7: update the error covariance*/
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return this-> angle;

}




void Kfilter::setAngle(double angle)
{
    this-> angle = angle;
}



double Kfilter::getRate()
{
    return this->rate;
}

/***** tune *******/
void Kfilter::setQangle(double Q_angle)
{
    this->Q_angle = Q_angle;
}

void Kfilter::setQbias(double Q_bias)
{
    this->Q_bias = Q_bias;
}

void Kfilter::setRmeasure(double R_measure)
{
    this->R_measure = R_measure;
}


double Kfilter::getQangle()
{
    return this->Q_angle;
}

double Kfilter::getQbias()
{
    return this->Q_bias;
}

double Kfilter::getRmeasure()
{
    return this->R_measure;
}


