#pragma once
#include <iostream>
#include <cmath>
#include <boost\math\quaternion.hpp>
#include <boost\numeric\ublas\vector.hpp>
using namespace std;
using namespace boost::math;
using namespace boost::numeric::ublas;

template<typename T>
quaternion<T> q_diff(quaternion<T> &q, quaternion<T> &AngVel)
{
    quaternion<T> dq;
    dq=0.5 * q * AngVel;
    return dq;
}
template<typename T>
quaternion<T> q_normalize(quaternion<T> q)
{
    quaternion<T> q_nor;
    q_nor = q / boost::math::abs(q);
    return q_nor;
}
quaternion<double> PureQuater_dot(quaternion<double> A, quaternion<double> B)
{  
    double b;
    double c;
    double d;
    b = A.R_component_2() * B.R_component_2();
    c = A.R_component_3() * B.R_component_3();
    d = A.R_component_4() * B.R_component_4();
    quaternion<double> result(0, b, c, d);
    return result;
}
quaternion<double> PureQuater_cross(quaternion<double> A, quaternion<double> B)
{  
    double b;
    double c;
    double d;
    b = A.R_component_3() * B.R_component_4() - A.R_component_4() * B.R_component_3();
    c = A.R_component_2() * B.R_component_4() - A.R_component_4() * B.R_component_2();
    d = A.R_component_2() * B.R_component_3() - A.R_component_3() * B.R_component_2();
    quaternion<double> result(0, b, c, d);
    return result;
}