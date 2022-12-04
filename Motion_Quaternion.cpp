#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <boost\math\quaternion.hpp>
#include "RigidBody.hpp"
#include "Force.hpp"
#include "q_operation.h"
using std::string;
using namespace boost::math;

void test01()
{
    int Iteration_num = 400;
    double time_step=0.01;
    double time = 0;
    string Rigid_FileName = "Rigid_Properties.txt";
    string Force_FileName = "Force_Properties.txt";
    string disp_outputFileName = "RigidBody_displacement.txt";
    string dynamic_outputFileName = "RigidBody_dynamic.txt";
    RigidBody R1;
    R1.Motion(time, time_step, Iteration_num, Force_FileName, Rigid_FileName,
     disp_outputFileName, dynamic_outputFileName);
}

int main()
{
    test01();
    return 0;
}


void RigidBody::Motion(double &time, double time_step, int Iteration_num,
 string Force_FileName, string Rigid_FileName, string disp_outputFileName, string dynamic_outputFileName)
{
    ofstream ofs_disp;
    ofstream ofs_dynamic;
    ofs_disp.open(disp_outputFileName, ios::out);
    ofs_dynamic.open(dynamic_outputFileName, ios::out);

    this->m_F1.getForceInfo(Force_FileName);
    this->getRigidInfo(Rigid_FileName);

    //输出文件第一行内容
    ofs_disp << left << setw(2) << "time"
             << "\t" << "X_Coordinate"
             << "\t" << "Y_Coordinate"
             << "\t" << "Z_Coordinate" << std::endl;
    ofs_dynamic << left << setw(18) << "time"
                << left << setw(29) << "Acceleration"<< "\t"
                << left << setw(25) << "Velocity"
                << left << setw(30) << "Angel_Acceleration"<< "\t"
                << "Angel_Velocity" << std::endl;

    //开始计算每步参数
    for (int i = 0; i < Iteration_num; i++)
    {
        this->m_F1.getForceInfo(Force_FileName);
        this->getTotal_Force();
        this->getNewAcc(this->m_AngAcc);
        this->getNewAngVel(time, time_step);
        this->getTranslation(time, time_step);
        this->getRotation_RungeKutta(time, time_step);
        time = time + time_step;
        this->m_AngVel[0] = this->m_AngVel[2];

        //输出位移至文件
        ofs_disp << fixed << setprecision(4) << time << "s";
        for (int j = 0; j < this->m_Rnum; j++)
        {
            // if(j != 0)
            // {
            //     ofs_disp << "\t" << left << setw(1);
            // }
            ofs_disp << fixed << setprecision(7)
                     << left << setw(2)
                     << "\t" << left << setw(14) << this->m_GlobalCoordinate[j].R_component_2()
                     << "\t" << left << setw(14) << this->m_GlobalCoordinate[j].R_component_3()
                     << "\t" << this->m_GlobalCoordinate[j].R_component_4() << std::endl;
        }
        ofs_disp << std::endl;

        //输出动力学参数至文件
        ofs_dynamic << fixed << setprecision(4) << time << left << setw(6) << "s"
                    << fixed << setprecision(5)
                    << "(" << this->m_Acc[0] << "," << this->m_Acc[1] << "," << this->m_Acc[2] <<  left << setw(5) << ")"
                    << "(" << this->m_Vel[0] << "," << this->m_Vel[1] << "," << this->m_Vel[2] << left << setw(5) << ")"
                    << "(" << this->m_AngAcc.R_component_2() << "," << this->m_AngAcc.R_component_3() << "," << this->m_AngAcc.R_component_4() << left << setw(5) << ")"
                    << "(" << this->m_AngVel[0].R_component_2() << "," << this->m_AngVel[0].R_component_3() << "," << this->m_AngVel[0].R_component_4() << ")" << std::endl;
    }
    ofs_dynamic.close();
    ofs_disp.close();
}

void Force::getForceInfo(string FileName)
{
    double pureQ_temp[3];
    double Fposition_temp;
    this->m_Fnum = 0;
    ifstream ifs;
    ifs.open(FileName, ios::in);
    if (!ifs.is_open())
	{
		std::cout << "文件打开失败" << std::endl;
		return;
	}
    string buf;
    std::vector<string> str;
    getline(ifs, buf);
    while(getline(ifs,buf))
    {
        this->m_Fnum++;
        str.push_back(buf);

    }
    int temp = 0;
    this->m_Force = new quaternion<double>[this->m_Fnum];
    this->m_Fposition = new int[this->m_Fnum];
    for (std::vector<string>::iterator it = str.begin(); it != str.end(); it++) 
    {
        std::istringstream s2n_Force(*it);
        s2n_Force >> Fposition_temp >> pureQ_temp[0] >> pureQ_temp[1] >> pureQ_temp[2];
        this->m_Force[temp] = quaternion<double>(0, pureQ_temp[0], pureQ_temp[1], pureQ_temp[2]);
        this->m_Fposition[temp] = Fposition_temp;
        temp++;
    }
}

void RigidBody::getRigidInfo(string FileName)
{
    this->m_Rnum = 0;
    double pureQ_temp[3];

    ifstream ifs;
    ifs.open(FileName, ios::in);
    if (!ifs.is_open())
	{
		std::cout << "文件打开失败" << std::endl;
		return;
	}
    string buf;
    //读取刚体质量
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_Mass(buf);
    s2n_Mass >> this->Mass;
    //读取刚体转动惯量
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_Inertia(buf);
    s2n_Inertia >> this->m_Inertia[0] >>this-> m_Inertia[1] >> this->m_Inertia[2];
    //读取刚体初始速度
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_Vel(buf);
    s2n_Vel >> this->m_Vel[0] >>this-> m_Vel[1] >> this->m_Vel[2];
    //读取刚体初始加速度
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_Acc(buf);
    s2n_Acc >> this->m_Acc[0] >>this-> m_Acc[1] >> this->m_Acc[2];
    //读取刚体初始角速度
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_AngVel(buf);
    s2n_AngVel >> pureQ_temp[0] >> pureQ_temp[1] >> pureQ_temp[2];
    this->m_AngVel[0] = quaternion<double>(0, pureQ_temp[0], pureQ_temp[1], pureQ_temp[2]);
    //读取刚体初始角加速度
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_AngAcc(buf);
    s2n_AngAcc >> pureQ_temp[0] >> pureQ_temp[1] >> pureQ_temp[2];
    this->m_AngAcc = quaternion<double>(0, pureQ_temp[0], pureQ_temp[1], pureQ_temp[2]);
    //读取刚体质心位置
    getline(ifs, buf);
    getline(ifs, buf);
    std::istringstream s2n_Center(buf);
    s2n_Center >> pureQ_temp[0] >> pureQ_temp[1] >> pureQ_temp[2];
    this->m_Center = quaternion<double>(0, pureQ_temp[0], pureQ_temp[1], pureQ_temp[2]);
    //读取刚体初始位置
    std::vector<string> str;
    getline(ifs, buf);
    while(getline(ifs,buf))
    {
        this->m_Rnum++;
        str.push_back(buf);
    }
    int temp = 0;
    this->m_LocalCoordinate = new quaternion<double> [this->m_Rnum];
    this->m_GlobalCoordinate = new quaternion<double>[this->m_Rnum];
    this->m_new_LocalCoordinate = new quaternion<double> [this->m_Rnum];
    for (std::vector<string>::iterator it = str.begin(); it != str.end(); it++) 
    {
        std::istringstream s2n_Coordinate(*it);
        s2n_Coordinate >> pureQ_temp[0] >> pureQ_temp[1] >> pureQ_temp[2];
        this->m_GlobalCoordinate[temp] = quaternion<double>(0, pureQ_temp[0], pureQ_temp[1], pureQ_temp[2]);
        this->m_LocalCoordinate[temp] = this->m_GlobalCoordinate[temp] - this->m_Center;
        temp++;
    }
}

void RigidBody::getTotal_Force()
{
    this->m_F1.m_Moment = quaternion<double>(0, 0, 0, 0);
    for (int i = 0; i < 3; i++)
    {
        this->m_F1.m_TotalForce[i] = 0;
    }
    double temp1, temp2, temp3;
    for (int i = 0; i < this->m_F1.m_Fnum; i++)
    {
        temp1 = this->m_F1.m_Force->R_component_2();
        temp2 = this->m_F1.m_Force->R_component_3();
        temp3 = this->m_F1.m_Force->R_component_4();
        this->m_F1.m_TotalForce[0] += temp1;
        this->m_F1.m_TotalForce[1] += temp2;
        this->m_F1.m_TotalForce[2] += temp3;
        this->m_F1.m_Moment += PureQuater_cross(this->m_LocalCoordinate[this->m_F1.m_Fposition[i]], *this->m_F1.m_Force);
    }
}

void RigidBody::getNewAcc(quaternion<double> &AngAcc)
{
    double temp;
    double AngAcc1_temp, AngAcc2_temp, AngAcc3_temp;

    this->m_Acc[0] = this->m_F1.m_TotalForce[0] / this->Mass;
    this->m_Acc[1] = this->m_F1.m_TotalForce[1] / this->Mass;
    this->m_Acc[2] = this->m_F1.m_TotalForce[2] / this->Mass;

    temp = 1 / m_Inertia[0];
    AngAcc1_temp = temp * (this->m_F1.m_Moment.R_component_2() - (m_Inertia[2] - 
    m_Inertia[1]) * m_AngVel[0].R_component_3() * m_AngVel[0].R_component_4());

    temp = 1 / m_Inertia[1];
    AngAcc2_temp = temp * (this->m_F1.m_Moment.R_component_3() - (m_Inertia[0] - 
    m_Inertia[2]) * m_AngVel[0].R_component_2() * m_AngVel[0].R_component_4());

    temp = 1 / m_Inertia[2];
    AngAcc3_temp = temp * (this->m_F1.m_Moment.R_component_4() - (m_Inertia[1] - 
    m_Inertia[0]) * m_AngVel[0].R_component_3() * m_AngVel[0].R_component_2());

    AngAcc = quaternion<double>(0, AngAcc1_temp, AngAcc2_temp, AngAcc3_temp);
}

void RigidBody:: getNewAngVel(double time, double time_step)
{
    double alpha;
    quaternion<double> AngAcc_temp;
    alpha = 0.5;
    //计算角速度
    //下一时刻加速度，暂用该时刻加速度代替
    AngAcc_temp = this->m_AngAcc;
    //this->getNewAngAcc(AngAcc_temp);
    this->m_AngVel[1] = this->m_AngVel[0] + ((1. - alpha) * m_AngAcc + alpha * AngAcc_temp) * 0.5 * time_step;
    this->m_AngVel[2] = this->m_AngVel[0] + ((1. - alpha) * m_AngAcc + alpha * AngAcc_temp) * time_step;
    //this->m_AngAcc = AngAcc_temp;
    
}

void RigidBody::getTranslation(double time, double time_step)
{
    //NewMark-beta法计算线速度与线位移
    double alpha, beta;
    alpha = 0.5;
    beta = 1 / 6.;
    double Acc_temp[3];
    double coordinate_temp1, coordinate_temp2, coordinate_temp3;
    //计算线速度
    //下一时刻线加速度，暂用该时刻代替
    for (int i = 0; i < 3; i++)
    {
        Acc_temp[i] = this->m_Acc[i];
    }
    this->m_Vel[0] += ((1. - alpha) * this->m_Acc[0] + alpha * Acc_temp[0]) * time_step;
    this->m_Vel[1] += ((1. - alpha) * this->m_Acc[1] + alpha * Acc_temp[1]) * time_step;
    this->m_Vel[2] += ((1. - alpha) * this->m_Acc[2] + alpha * Acc_temp[2]) * time_step;

    //更新质心位置
    coordinate_temp1 = this->m_Center.R_component_2() + this->m_Vel[0] * time_step 
    + ((0.5 - beta) * this->m_Acc[0] + beta * Acc_temp[0]) * time_step;
    coordinate_temp2 = this->m_Center.R_component_3()+this->m_Vel[1] * time_step 
    + ((0.5 - beta) * this->m_Acc[1] + beta * Acc_temp[1]) * time_step;
    coordinate_temp3 = this->m_Center.R_component_4()+this->m_Vel[2] * time_step 
    + ((0.5 - beta) * this->m_Acc[2] + beta * Acc_temp[2]) * time_step;
    this->m_Center = quaternion<double>(0, coordinate_temp1, coordinate_temp2, coordinate_temp3);

    //更新各质点位置
    for (int i = 0; i < this->m_Rnum; i++)
    {
        coordinate_temp1 = this->m_GlobalCoordinate[i].R_component_2() + this->m_Vel[0] * time_step 
        + ((0.5 - beta) * this->m_Acc[0] + beta * Acc_temp[0]) * time_step;
        coordinate_temp2 = this->m_GlobalCoordinate[i].R_component_3()+this->m_Vel[1] * time_step 
        + ((0.5 - beta) * this->m_Acc[1] + beta * Acc_temp[1]) * time_step;
        coordinate_temp3 = this->m_GlobalCoordinate[i].R_component_4()+this->m_Vel[2] * time_step 
        + ((0.5 - beta) * this->m_Acc[2] + beta * Acc_temp[2]) * time_step;
        this->m_GlobalCoordinate[i] = quaternion<double>(0, coordinate_temp1, coordinate_temp2, coordinate_temp3);
    }
}

void RigidBody::getRotation_RungeKutta(double time, double time_step)
{
    quaternion<double> K1, K2, K3, K4;
    quaternion<double> q1, q2, q3;
    quaternion<double> AngVel_temp;
    quaternion<double> qc;
    this->m_q = q_normalize(this->m_q);
    K1 = q_diff(this->m_q, this->m_AngVel[2]);
    q1 = q_normalize(this->m_q + 0.5 * time_step * K1);

    //AngVel(time+0.5*time_step)
    K2 = q_diff(q1, this->m_AngVel[1]);
    q2 = q_normalize(this->m_q + 0.5 * time_step * K2);

    K3 = q_diff(q2, this->m_AngVel[1]);
    q3 = q_normalize(this->m_q + time_step * K3);

    K4 = q_diff(q3, this->m_AngVel[1]);
    this->m_q = this->m_q + time_step / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
    this->m_q = q_normalize(this->m_q);

    qc = conj(this->m_q);
    for (int i = 0; i < this->m_Rnum; i++)
    {
        this->m_LocalCoordinate[i] = this->m_GlobalCoordinate[i] - this->m_Center;
        this->m_new_LocalCoordinate[i] = this->m_q * this->m_LocalCoordinate[i] * qc;
        this->m_GlobalCoordinate[i] = this->m_new_LocalCoordinate[i] + this->m_Center;
    }

}



