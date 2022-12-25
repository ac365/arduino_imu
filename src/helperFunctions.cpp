#include "helperFunctions.h"

Matrix<4> HelperFunctions::euler2Quaternion(Matrix<3> eul)
{
    //re-package euler angles
    float phi  =eul(0,0);
    float theta=eul(1,0);
    float psi  =eul(2,0);

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

Matrix<3> HelperFunctions::quaternion2Euler(Matrix<4> q)
{
    //re-package quaternions
    float q0 = q(0);
    float q1 = q(1);
    float q2 = q(2);
    float q3 = q(3);

    //convert to euler angles
    Matrix<3> eul;
    eul(0)=atan2(2*(q1*q2 + q0*q3),(q0*q0 + q1*q1 - q2*q2 - q3*q3));//psi
    eul(1)=asin(-2*(q1*q3 - q0*q2));                                //theta
    eul(2)=atan2(2*(q2*q3 + q0*q1),q0*q0 - q1*q1 - q2*q2 + q3*q3);  //phi
    
    return eul;
}