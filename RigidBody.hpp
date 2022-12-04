#pragma once
#include <iostream>
#include <string>
#include <boost\math\quaternion.hpp>
#include "Force.hpp"
using std::string;
using namespace boost::math;

class RigidBody
{
    public:
        RigidBody()
        {
            this->m_Rnum = 0;
            this->m_LocalCoordinate = new quaternion<double> [this->m_Rnum];
            this->m_new_LocalCoordinate = new quaternion<double> [this->m_Rnum];
            this->m_GlobalCoordinate = new quaternion<double> [this->m_Rnum];
            this->m_q = quaternion<double>(1, 0., 0., 0);
        }
        RigidBody(int Rnum)
        {
            this->m_Rnum = Rnum;
            this->m_LocalCoordinate = new quaternion<double> [this->m_Rnum];
            this->m_new_LocalCoordinate = new quaternion<double> [this->m_Rnum];
            this->m_GlobalCoordinate = new quaternion<double> [this->m_Rnum];
            this->m_q = quaternion<double>(1, 0., 0., 0);
        }
        RigidBody(const RigidBody &r)
        {
            this->m_Rnum = r.m_Rnum;
            this->m_LocalCoordinate = new quaternion<double>(*r.m_LocalCoordinate);
            this->m_new_LocalCoordinate = new quaternion<double>(*r.m_new_LocalCoordinate);
            this->m_GlobalCoordinate = new quaternion<double>(*r.m_GlobalCoordinate);
            this->m_q = quaternion<double>(1, 0., 0., 0);
        }
        //获取刚体参数信息
        void getRigidInfo(string FileName);
        //计算刚体所受合力
        void getTotal_Force();
        //计算角加速度与线加速度
        void getNewAcc(quaternion<double> &AngAcc);
        //计算角速度
        void getNewAngVel(double time, double time_step);
        //同时计算线速度与线位移
        void getTranslation(double time, double time_step);
        //计算角位移
        void getRotation_RungeKutta(double time, double time_step);
        //直接调用，用于计算刚体位移
        void Motion(double &time, double time_step, int Iteration_num, 
        string Force_FileName, string Rigid_FileName, string disp_outputFileName, string dynamic_outputFileName);
        ~RigidBody()
        {
            if (m_LocalCoordinate != NULL)
		    {
			    delete[] m_LocalCoordinate;
                m_LocalCoordinate = NULL;
            }
            if (m_new_LocalCoordinate != NULL)
		    {
			    delete[] m_new_LocalCoordinate;
                m_new_LocalCoordinate = NULL;
            }
            if (m_GlobalCoordinate != NULL)
		    {
			    delete[] m_GlobalCoordinate;
                m_GlobalCoordinate = NULL;
            }
        }
    private:
        // m_Rnum:刚体所拥有的质点数
        // Mass:刚体质量
        // m_Inertia:刚体转动惯量
        // m_Vel[3]:刚体本时刻线速度的三个分量
        // m_Acc[3]:刚体本时刻线加速度的三个分量
        // m_Center:刚体质心，采用纯四元数表示
        // m_AngVel[3]:三个分量分别代表刚体本时刻、t+0.5*delta_t时刻、t+delta_t时刻角速度，采用纯四元数表示
        // m_AngAcc:刚体本时刻角加速度，采用纯四元数表示
        // m_q:旋转用四元数，初始值为（1，0，0，0)
        // *m_LocalCoordinate:本时刻刚体各质点在质心坐标系下的局部坐标
        // *m_new_LocalCoordinate:t+delta_t时刻刚体各质点在质心坐标系下的局部坐标
        // *m_GlobalCoordinate:本时刻刚体各质点在整体坐标系下的整体坐标
        int m_Rnum;
        double Mass;
        double m_Inertia[3];
        double m_Vel[3];
        double m_Acc[3];
        quaternion<double> m_Center;
        quaternion<double> m_AngVel[3];
        quaternion<double> m_AngAcc;
        quaternion<double> m_q;
        quaternion<double> *m_LocalCoordinate;
        quaternion<double> *m_new_LocalCoordinate;
        quaternion<double> *m_GlobalCoordinate;
        Force m_F1;
};