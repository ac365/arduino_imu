#include "BasicLinearAlgebra.h"
using namespace BLA;

class HelperFunctions
{
public:
    float g = 9.80665;

    Matrix<4> euler2Quaternion(Matrix<3> eul);
    Matrix<3> quaternion2Euler(Matrix<4> q);

private:

};