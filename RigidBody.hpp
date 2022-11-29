#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <boost\math\quaternion.hpp>
#include <boost\numeric\ublas\vector.hpp>
using namespace std;
using namespace boost::math;
using namespace boost::numeric::ublas;
class RigidBody
{
    public:
        RigidBody(quaternion<double> *Position, int num)
        {
            this->m_num = num;
            this->m_Position = new quaternion<double> [this->m_num];
            for (int i = 0; i < this->m_num; i++)
            {
                this->m_Position[i] = Position[i];
            }      
        }
        void getMoment(double **Force, int *F_Position, int num);
        void getAngAcc();
        void getAngVel(double time);
        void getQ_RungeKutta(quaternion<double> &q, double time, double time_step);
        void Moving();
        ~RigidBody()
        {
            delete[] m_Position;
        }
    public:
        int m_num;
        double m_Inertia[3];
        quaternion<double> m_Center;
        quaternion<double> m_Moment;
        quaternion<double> m_AngVel;
        quaternion<double> m_AngAcc;
        quaternion<double> *m_Position;
};