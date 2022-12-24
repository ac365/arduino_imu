#include "kalmanFilter.h"
#include "BasicLinearAlgebra.h"
#include "helperFunctions.h"

using namespace BLA;
HelperFunctions hlp;

void KalmanFilter::Initialize(float kalmanP, float kalmanQ, float kalmanR)
{
    //state vector
    x = hlp.euler2Quaternion(Matrix<3> {0.0, 0.0, 0.0});

    //error covariance matrix
    P(0,0)=kalmanP; P(0,1)=0.0;     P(0,2)=0.0;     P(0,3)=0.0;
    P(1,0)=0.0;     P(1,1)=kalmanP; P(1,2)=0.0;     P(1,3)=0.0;
    P(2,0)=0.0;     P(2,1)=0.0;     P(2,2)=kalmanP; P(2,3)=0.0;
    P(3,0)=0.0;     P(3,1)=0.0;     P(3,2)=0.0;     P(3,3)=kalmanP;

    //model noise
    Q(0,0)=kalmanQ; Q(0,1)=0.0;     Q(0,2)=0.0;     Q(0,3)=0.0;
    Q(1,0)=0.0;     Q(1,1)=kalmanQ; Q(1,2)=0.0;     Q(1,3)=0.0;
    Q(2,0)=0.0;     Q(2,1)=0.0;     Q(2,2)=kalmanQ; Q(2,3)=0.0;
    Q(3,0)=0.0;     Q(3,1)=0.0;     Q(3,2)=0.0;     Q(3,3)=kalmanQ;

    //sensor noise covariance
    R(0,0)=kalmanR; R(0,1)=0.0;     R(0,2)=0.0;     R(0,3)=0.0;
    R(1,0)=0.0;     R(1,1)=kalmanR; R(1,2)=0.0;     R(1,3)=0.0;
    R(2,0)=0.0;     R(2,1)=0.0;     R(2,2)=kalmanR; R(2,3)=0.0;
    R(3,0)=0.0;     R(3,1)=0.0;     R(3,2)=0.0;     R(3,3)=kalmanR;
}

void KalmanFilter::_Predict(sensors_event_t g, float dt)
{
    //re-package angular velocity measurements
    Matrix<3> w = {g.gyro.x, g.gyro.y, g.gyro.z};
    float p=w(0);
    float q=w(1);
    float r=w(2);

    //predict state
    Matrix<4,4> stm; //state transition matrix
    stm(0,0)=0.0; stm(0,1)=-p;  stm(0,2)=-q;  stm(0,3)=-r;
    stm(1,0)= p;  stm(1,1)=0.0; stm(1,2)= r;  stm(1,3)=-q;
    stm(2,0)= q;  stm(2,1)=-r;  stm(2,2)=0.0; stm(2,3)= p;
    stm(3,0)= r;  stm(3,1)= q;  stm(3,2)=-p;  stm(3,3)=0.0;
    stm *= 0.5;

    x += stm*x;

    //update error covariance
    Matrix<4,4> A; //Jacobian of state equation
    A = stm;

    P += (A*P + P*(~A) + Q)*dt;
}

void KalmanFilter::_Update(sensors_event_t a)
{
    //re-package acceleration measurements and state
    Matrix<3> y = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    float q0 = x(0);
    float q1 = x(1);
    float q2 = x(2);    
    float q3 = x(3);    

    //accelerometer model
    Matrix<3> h;
    h(0) = 2*(q1*q3 - q0*q2);
    h(1) = 2*(q2*q3 + q0*q1);
    h(2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    h   *= -hlp.g;

    //accelerometer model Jacobian
    Matrix<3,4> C;
    C(0,0)=-q2; C(0,1)= q3; C(0,2)=-q0; C(0,3)=q1;
    C(1,0)= q1; C(1,1)= q0; C(0,2)= q3; C(1,3)=q2;
    C(2,0)= q0; C(2,1)=-q1; C(0,2)=-q2; C(2,3)=q3;

    //compute Kalman gain
    Matrix<4,3> K;
    Matrix<3,3> tmp;
    tmp = C*P*(~C)+R;
    Invert(tmp);
    K = P*(~C)*tmp;

    //correct state estimate predictions
    x += K*(y - h);

    //update error covariance matrix
    Matrix<4,4> I;
    I(0,0)=1.0; I(0,1)=0.0; I(0,2)=0.0; I(0,3)=0.0;
    I(1,0)=0.0; I(1,1)=1.0; I(1,2)=0.0; I(1,3)=0.0;
    I(2,0)=0.0; I(2,1)=0.0; I(2,2)=1.0; I(2,3)=0.0;
    I(3,0)=0.0; I(3,1)=0.0; I(3,2)=0.0; I(3,3)=1.0;
    
    P = (I - K*C)*P;
};

Matrix<4> KalmanFilter::Step(sensors_event_t gyro, sensors_event_t accel, float dt)
{
    KalmanFilter::_Predict(gyro,dt);
    KalmanFilter::_Update(accel);

    return x;
};