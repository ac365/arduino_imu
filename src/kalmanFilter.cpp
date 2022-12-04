#include "kalmanFilter.h"
#include "BasicLinearAlgebra.h"
#include "helperFunctions.h"

using namespace BLA;
HelperFunctions hlp;

void KalmanFilter::Initialize(float kalmanP, float kalmanQ, float kalmanR)
{
    //state matrix
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

void KalmanFilter::Predict(sensors_event_t g, float dt)
{
    //re-package angular velocity
    Matrix<3> w = {g.gyro.x, g.gyro.y, g.gyro.z};
    float p=w(0);
    float q=w(1);
    float r=w(2);

    //predict state
    Matrix<4,4> stm; //state transition matrix
    stm(0,0)=0.0; stm(0,1)=-p;  stm(0,2)=-q;  stm(0,3)=-r;
    stm(1,0)=p;   stm(1,1)=0.0; stm(1,2)= r;  stm(1,3)=-q;
    stm(2,0)=q;   stm(2,1)=-r;  stm(2,2)=0.0; stm(2,3)= p;
    stm(3,0)=r;   stm(3,1)= q;  stm(3,2)=-p;  stm(3,3)=0.0;
    stm *= 0.5;

    x += stm*x;

    //update error covariance
    Matrix<4,4> A; //Jacobian of state equation
    A = stm;

    P += (A*P + P*(~A) + Q)*dt;
}