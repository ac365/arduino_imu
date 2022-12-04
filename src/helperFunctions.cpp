#include "helperFunctions.h"

Matrix<4,1> HelperFunctions::euler2Quaternion(Matrix<3,1> eulerAngles)
{
    //re-package euler angles
    float phi  =eulerAngles(0,0);
    float theta=eulerAngles(1,0);
    float psi  =eulerAngles(2,0);

    //convert to quaternions
    float cp=cos(0.5*phi);   float sp=sin(0.5*phi);
    float ct=cos(0.5*theta); float st=sin(0.5*theta);    
    float cs=cos(0.5*psi);   float ss=sin(0.5*psi);

    Matrix<4> q;
    q(0)=cs*ct*cp + ss*st*sp;
    q(1)=cs*ct*sp - ss*st*cp;
    q(2)=cs*st*sp + ss*ct*sp;
    q(3)=ss*ct*cp - cs*st*sp;

    return q;
}