#pragma once
#include <iostream>
#include <string>
#include <boost\math\quaternion.hpp>
using std::string;
using namespace boost::math;

class Force
{
    friend class RigidBody;
    public:
        Force()
        {
            this->m_Fnum = 0;
            this->m_Force = new quaternion<double>[this->m_Fnum];
            this->m_Fposition = new int[this->m_Fnum];
        }
        Force(int Fnum)
        {
            this->m_Fnum = Fnum;
            this->m_Force = new quaternion<double>[this->m_Fnum];
            this->m_Fposition = new int[this->m_Fnum];
        }
        Force(const Force &f)
        {
            this->m_Fnum = f.m_Fnum;
            this->m_Force = new quaternion<double>(*f.m_Force);
            this->m_Fposition = new int(*f.m_Fposition);
        }
        //获取刚体受力信息
        void getForceInfo(string FileName);
        ~Force()
        {
            if (m_Fposition != NULL)
		    {
			    delete[] m_Fposition;
                m_Fposition = NULL;
            }
            if (m_Force != NULL)
		    {
			    delete[] m_Force;
                m_Force = NULL;
            }    
        }
    private:
        // m_Fnum:刚体所受力的数目
        // *m_Fposition:各力所施加的刚体质点编号
        // *m_Force:刚体受力大小
        // m_Moment:将刚体所受力移至质心时所产生的合力矩
        // m_TotalForce:将刚体所受力移至质心时所产生的合力
        int m_Fnum;
        int *m_Fposition;
        quaternion<double> *m_Force;
        quaternion<double> m_Moment;
        double m_TotalForce[3] = {0};
};