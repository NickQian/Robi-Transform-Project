/*
    Falman Filter which mainly reference "github.com/TKJElectronics"
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.4.26  init version  - ref: github.com/TKJElectonics/KalmanFilter
    ----
*/


#ifndef _KFILTER_H_
#define _KFILTER_H_

class Kfilter{

 public:

    Kfilter(double cfg_Q_angle, double cfg_Q_bias, double cfg_R_measure);
    //Kfilter();
    ~Kfilter();

    double getAngle(double newAngle, double newRate, double dt);   // degree, degree/s, delta_time(second)
    void  setAngle(double angle);
    double getRate();

    /* Tune the filter */
    void setQangle(double Q_angle);
    void setQbias(double Q_bias);           // raise to follow input more closely. lower to smooth output
    void setRmeasure(double newR_measure);

    /* Get values */
    double getQangle();
    double getQbias();
    double getRmeasure();


    double Q_angle;          // for accelermeter
    double Q_bias;           // for gyro bias
    double R_measure;        // Measurement noise variance

// protected:

 private:
    double angle;            // angle from kalman filter
    double bias;             // gyro bias from kalman filter
    double rate;             // unbiased rate calculated from the rate and the bias

    double P[2][2];          // error covariance matrix
    double K[2];             // Kalman gian
    double y;                // Angle diff
    double S;                // estimate error
};

#endif




