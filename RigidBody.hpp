#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <boost\math\quaternion.hpp>
using namespace std;
using namespace boost::math;

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
        void getInertia(double *Inertia);
        void getMoment(double (*Force)[3], int *F_Position, int num);
        quaternion<double> getAngAcc();
        void NewMark(double time, double time_step, quaternion<double> &q);
        //void getAngVel(double time);
        //void getQ_RungeKutta(quaternion<double> &q, double time, double time_step);
        void Moving(double (*Force)[3], int *F_Position, int F_num);
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