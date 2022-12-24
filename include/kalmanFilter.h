#include "BasicLinearAlgebra.h"
#include <Adafruit_Sensor.h>
using namespace BLA;



class KalmanFilter
{    
public:
    void Initialize(float kalmanP, float kalmanQ, float kalmanR);
    void Predict(sensors_event_t gyro, float dt);
    void Update(sensors_event_t accel, float dt);

private:
    Matrix<4>   x;
    Matrix<4,4> P;
    Matrix<4,4> Q;
    Matrix<3,3> R;
};