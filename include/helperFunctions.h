#include "BasicLinearAlgebra.h"
using namespace BLA;

class HelperFunctions
{
public:
    float g = 9.80665;

    Matrix<4,1> euler2Quaternion(Matrix<3,1> eulerAngles);

private:

};