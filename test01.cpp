#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <boost\math\quaternion.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include "RigidBody.hpp"
#include "q_operation.h"
using namespace std;
using namespace boost::math;
using namespace boost::numeric::ublas;

void RigidBody::getMoment(double **Force, int *F_Position, int num)
{
    m_Moment = quaternion<double>(0, 0, 0, 0);
    quaternion<double> Q_Force;
    for (int i = 0; i < num; i++)
    {
        Q_Force = quaternion<double>(0, Force[i][0], Force[i][1], Force[i][2]);
        m_Moment += PureQuater_cross(m_Position[F_Position[i]], Q_Force);
    }
}
void RigidBody::getAngAcc()
{
    m_Moment.R_component_2() - m_Inertia[0];
}
void RigidBody:: getAngVel(double time)
{
    double theta0 = M_PI / 6;
    double T = 1;
    double w = 2 * M_PI / T;
    double theta = theta0 * sin(w * time);
    double dtheta = theta0 * w * cos(w * time);
    this->m_AngVel = quaternion<double>(0, 0, 0, dtheta);
    //this->m_AngVel = quaternion<double>(0, 0, 0, 0.5*M_PI);
    
}

void RigidBody::getQ_RungeKutta(quaternion<double> &q, double time, double time_step)
{
    quaternion<double> K1, K2, K3, K4;
    quaternion<double> q1, q2, q3;
    q = q_normalize(q);
    K1 = q_diff(q, this->m_AngVel);
    q1 = q_normalize(q + 0.5 * time_step * K1);

    //AngVel(time+0.5*time_step)
    RigidBody::getAngVel(time + 0.5 * time_step);
    K2 = q_diff(q1, this->m_AngVel);
    q2 = q_normalize(q + 0.5 * time_step * K2);

    K3 = q_diff(q2, this->m_AngVel);
    q3 = q_normalize(q + time_step * K3);

    K4 = q_diff(q3, this->m_AngVel);
    q = q + time_step / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
    q = q_normalize(q);
}
void RigidBody::Moving()
{
    double time_step = 0.0025;
    double Iteration_num = 100;
    double time = 0.;
    quaternion<double> r1;
    quaternion<double> q(1, 0., 0., 0);
    quaternion<double> qc;
    for (int i = 0; i < Iteration_num; i++)
    {
        //cout << "step:" << i << endl;
        RigidBody::getAngVel(time);
        getQ_RungeKutta(q, time, time_step);
        qc = conj(q); 
        cout<<q<<endl;    
        for (int j = 0; j < this->m_num; j++)
        {
            r1 = q * this->m_Position[j] * qc;   
            cout << r1 << endl;
        }
        time += time_step;
        //cout << time << endl;
        cout << endl;
    }
}
void test01()
{
    int num = 4;
    quaternion<double> Position[4];
    Position[0] = quaternion<double>(0, 1, 0, 0);
    Position[1] = quaternion<double>(0, 2, 0, 0);
    Position[2] = quaternion<double>(0, 3, 0, 0);
    Position[3] = quaternion<double>(0, 4, 0, 0);
    RigidBody R1(Position, num);
    R1.Moving();
}
int main()
{
    test01();
    system("pause");
    return 0;
}
