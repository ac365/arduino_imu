#include "BasicLinearAlgebra.h"
#include <Adafruit_Sensor.h>
using namespace BLA;



class KalmanFilter
{    
public:
    const void      Initialize(float kalmanP, float kalmanQ, float kalmanR);
    const Matrix<4> Step(sensors_event_t gyro, sensors_event_t accel, float dt);

private:
    Matrix<4>   x;
    Matrix<4,4> P;
    Matrix<4,4> Q;
    Matrix<3,3> R;

    const void _Predict(sensors_event_t gyro, float dt);
    const void _Update(sensors_event_t accel);
};