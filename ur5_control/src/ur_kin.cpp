#include "ur_kin.h"
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>


namespace ur_kinematics {

  namespace {
    const double ZERO_THRESH = 0.00000001;
    int SIGN(double x) {
      return (x > 0) - (x < 0);
    }
    const double PI = M_PI;
    const double d1 =  0.089159;
    const double a2 = -0.42500;
    const double a3 = -0.39225;
    const double d4 =  0.10915;
    const double d5 =  0.09465;
    const double d6 =  0.0823;
  }

  void forward(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
    double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q); 
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
    *T = c234*c1*s5 - c5*s1; T++;
    *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
    *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
    *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
    *T = c1*c5 + c234*s1*s5; T++;
    *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
    *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
    *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
    *T = -s234*s5; T++;
    *T = -c234*s6 - s234*c5*c6; T++;
    *T = s234*c5*s6 - c234*c6; T++;
    *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  }

  void forward_all(const double* q, double* T1, double* T2, double* T3, 
                                    double* T4, double* T5, double* T6) {
    double s1 = sin(*q), c1 = cos(*q); q++; // q1
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
    q234 += *q; q++; // q4
    double s5 = sin(*q), c5 = cos(*q); q++; // q5
    double s6 = sin(*q), c6 = cos(*q); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);

    if(T1 != NULL) {
      *T1 = c1; T1++;
      *T1 = 0; T1++;
      *T1 = s1; T1++;
      *T1 = 0; T1++;
      *T1 = s1; T1++;
      *T1 = 0; T1++;
      *T1 = -c1; T1++;
      *T1 = 0; T1++;
      *T1 =       0; T1++;
      *T1 = 1; T1++;
      *T1 = 0; T1++;
      *T1 =d1; T1++;
      *T1 =       0; T1++;
      *T1 = 0; T1++;
      *T1 = 0; T1++;
      *T1 = 1; T1++;
    }

    if(T2 != NULL) {
      *T2 = c1*c2; T2++;
      *T2 = -c1*s2; T2++;
      *T2 = s1; T2++;
      *T2 =a2*c1*c2; T2++;
      *T2 = c2*s1; T2++;
      *T2 = -s1*s2; T2++;
      *T2 = -c1; T2++;
      *T2 =a2*c2*s1; T2++;
      *T2 =         s2; T2++;
      *T2 = c2; T2++;
      *T2 = 0; T2++;
      *T2 =   d1 + a2*s2; T2++;
      *T2 =               0; T2++;
      *T2 = 0; T2++;
      *T2 = 0; T2++;
      *T2 =                 1; T2++;
    }

    if(T3 != NULL) {
      *T3 = c23*c1; T3++;
      *T3 = -s23*c1; T3++;
      *T3 = s1; T3++;
      *T3 =c1*(a3*c23 + a2*c2); T3++;
      *T3 = c23*s1; T3++;
      *T3 = -s23*s1; T3++;
      *T3 = -c1; T3++;
      *T3 =s1*(a3*c23 + a2*c2); T3++;
      *T3 =         s23; T3++;
      *T3 = c23; T3++;
      *T3 = 0; T3++;
      *T3 =     d1 + a3*s23 + a2*s2; T3++;
      *T3 =                    0; T3++;
      *T3 = 0; T3++;
      *T3 = 0; T3++;
      *T3 =                                     1; T3++;
    }

    if(T4 != NULL) {
      *T4 = c234*c1; T4++;
      *T4 = s1; T4++;
      *T4 = s234*c1; T4++;
      *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
      *T4 = c234*s1; T4++;
      *T4 = -c1; T4++;
      *T4 = s234*s1; T4++;
      *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
      *T4 =         s234; T4++;
      *T4 = 0; T4++;
      *T4 = -c234; T4++;
      *T4 =                  d1 + a3*s23 + a2*s2; T4++;
      *T4 =                         0; T4++;
      *T4 = 0; T4++;
      *T4 = 0; T4++;
      *T4 =                                                  1; T4++;
    }

    if(T5 != NULL) {
      *T5 = s1*s5 + c234*c1*c5; T5++;
      *T5 = -s234*c1; T5++;
      *T5 = c5*s1 - c234*c1*s5; T5++;
      *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
      *T5 = c234*c5*s1 - c1*s5; T5++;
      *T5 = -s234*s1; T5++;
      *T5 = - c1*c5 - c234*s1*s5; T5++;
      *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
      *T5 =                           s234*c5; T5++;
      *T5 = c234; T5++;
      *T5 = -s234*s5; T5++;
      *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
      *T5 =                                                   0; T5++;
      *T5 = 0; T5++;
      *T5 = 0; T5++;
      *T5 =                                                                                 1; T5++;
    }

    if(T6 != NULL) {
      *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
      *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
      *T6 = c5*s1 - c234*c1*s5; T6++;
      *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
      *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
      *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
      *T6 = - c1*c5 - c234*s1*s5; T6++;
      *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
      *T6 =                                       c234*s6 + s234*c5*c6; T6++;
      *T6 = c234*c6 - s234*c5*s6; T6++;
      *T6 = -s234*s5; T6++;
      *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
      *T6 =                                                                                                   0; T6++;
      *T6 = 0; T6++;
      *T6 = 0; T6++;
      *T6 =                                                                                                                                            1; T6++;
    }
  }

  int inverse(const double* T, double* q_sols, double q6_des) {
    int num_sols = 0;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++; 
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++; 
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;
            q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k]; 
            q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k]; 
            q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6; 
            num_sols++;
          }

        }
      }
    }
    return num_sols;
  }
};


#define IKFAST_HAS_LIBRARY
#include "ikfast.h"
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==61);

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

void to_mat44(double * mat4_4, const IkReal* eetrans, const IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        mat4_4[i*4+0] = eerot[i*3+0];
        mat4_4[i*4+1] = eerot[i*3+1];
        mat4_4[i*4+2] = eerot[i*3+2];
        mat4_4[i*4+3] = eetrans[i];
    }
    mat4_4[3*4+0] = 0;
    mat4_4[3*4+1] = 0;
    mat4_4[3*4+2] = 0;
    mat4_4[3*4+3] = 1;
}

void from_mat44(const double * mat4_4, IkReal* eetrans, IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        eerot[i*3+0] = mat4_4[i*4+0];
        eerot[i*3+1] = mat4_4[i*4+1];
        eerot[i*3+2] = mat4_4[i*4+2];
        eetrans[i] = mat4_4[i*4+3];
    }
}


IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
  if(!pfree) return false;

  int n = GetNumJoints();
  double q_sols[8*6];
  double T[16];

  to_mat44(T, eetrans, eerot);

  int num_sols = ur_kinematics::inverse(T, q_sols,pfree[0]);

  std::vector<int> vfree(0);

  for (int i=0; i < num_sols; ++i){
    std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(n);
    for (int j=0; j < n; ++j) vinfos[j].foffset = q_sols[i*n+j];
    solutions.AddSolution(vinfos,vfree);
  }
  return num_sols > 0;
}

IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot)
{
    double T[16];
    ur_kinematics::forward(j,T);
    from_mat44(T,eetrans,eerot);
}

double rad_format(double data)
{
    while(data > 3.1415926)
        data -= 2*3.1415926;
    while(data < -3.1415926)
        data += 2*3.1415926;
    return data; 
}

IKFAST_API int GetNumFreeParameters() { return 1; }
IKFAST_API int* GetFreeParameters() { static int freeparams[] = {5}; return freeparams; }
IKFAST_API int GetNumJoints() { return 6; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN

using namespace std;
using namespace ur_kinematics;

#include "ur5_control/ur5_dke_pos.h"
#include "std_msgs/Float64MultiArray.h"
#include "ur5_control/ur5_pos.h"
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct f_t
{
    float fx;
    float fy;
    float fz;
    float tx;
    float ty;
    float tz;
};

f_t end_force;
ur5_control::ur5_pos ur5_pos_msg;
ur5_control::ur5_dke_pos ur5_dke_pos_msg;
std_msgs::Float64MultiArray current_pos;
std_msgs::Float64MultiArray target_pos;
float min_squre_error = 1000;
bool ifcommand = false;
double* T = new double[16];
double* T_cal = new double[16];

void poscommandcallback(const ur5_control::ur5_dke_pos::ConstPtr& msg)
{
    ur5_dke_pos_msg = *msg;
    // 构造四元数对象
    Eigen::Quaterniond quaternion(msg->qw, msg->qx, msg->qy, msg->qz);

    // 规范化四元数（确保其为单位四元数）
    quaternion.normalize();

    // 将四元数转化为旋转矩阵
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    // 构造变换矩阵 (4x4)
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix(0, 3) = msg->x;
    transformationMatrix(1, 3) = msg->y;
    transformationMatrix(2, 3) = msg->z;
    T[0] = transformationMatrix(0,0);
    T[1] = transformationMatrix(0,1);
    T[2] = transformationMatrix(0,2);
    T[3] = transformationMatrix(0,3);
    T[4] = transformationMatrix(1,0);
    T[5] = transformationMatrix(1,1);
    T[6] = transformationMatrix(1,2);
    T[7] = transformationMatrix(1,3);
    T[8] = transformationMatrix(2,0);
    T[9] = transformationMatrix(2,1);
    T[10] = transformationMatrix(2,2);
    T[11] = transformationMatrix(2,3);
    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;

    for(int k=0;k<16;k++)
    {
        T_cal[k] = T[k];
    }
    current_pos.data[0] = msg->arm1;
    current_pos.data[1] = msg->arm2;
    current_pos.data[2] = msg->arm3;
    current_pos.data[3] = msg->arm4;
    current_pos.data[4] = msg->arm5;
    current_pos.data[5] = msg->arm6;
    ifcommand = true;
    min_squre_error = 1000;
    std::cout<<"ok"<<std::endl;
}
void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 打印接收到的力和力矩
    // ROS_INFO("Received WrenchStamped:");
    end_force.fx = msg->wrench.force.x;
    end_force.fy = msg->wrench.force.y;
    end_force.fz = msg->wrench.force.z;
    end_force.tx = msg->wrench.torque.x;
    end_force.ty = msg->wrench.torque.y;
    end_force.tz = msg->wrench.torque.z;
    // ROS_INFO("  Force - x: %f, y: %f, z: %f", msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    // ROS_INFO("  Torque - x: %f, y: %f, z: %f", msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
}

float squre_error(float data1, float data2)
{
    return rad_format(data1-data2)*rad_format(data1-data2);
}

void admittance_control(f_t force)
{
    float y_add = 0.0f;
      if(fabs(force.fy) > 20)
      {
        y_add = force.fy / 10000;
        std::cout<<"recal"<<std::endl;
        ifcommand = true;
        T_cal[7] = T[7] + y_add;
        // std::cout<<T_cal[7]<<std::endl;
        std::cout<<"f:"<<force.fy<<"add_y:"<<y_add<<"target_y:"<<T_cal[7]<<std::endl; 
      }


}

int main(int argc, char* argv[])
{
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    ur5_pos_msg.arm1 = 0.0f;
    ur5_pos_msg.arm2 = 0.0f;
    ur5_pos_msg.arm3 = 0.0f;
    ur5_pos_msg.arm4 = 0.0f;
    ur5_pos_msg.arm5 = 0.0f;
    ur5_pos_msg.arm6 = 0.0f;
    ros::init(argc, argv, "UR_Kinematics");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("ur5_dke_pos", 1000, poscommandcallback);
    ros::Subscriber wrench_sub = nh.subscribe("/ft_sensor_topic", 1000, wrenchCallback);
    ros::Publisher ur5_pub = nh.advertise<ur5_control::ur5_pos>("ur5_pos", 1000);
    ros::Rate loop_rate(10); // 设置循环的频率，这里是10Hz
    while(ros::ok())
    {
        ros::spinOnce();
        if(ifcommand)
        {
            for(int i=0;i<4;i++) 
            {
                // for(int j=i*4;j<(i+1)*4;j++)
                // printf("%1.3f ", T[j]);
                // printf("\n");
            }
            // std::cout<<"here"<<std::endl;
            double q_sols[8*6];
            int num_sols;
            num_sols = inverse(T_cal, q_sols);
            // for(int i=0;i<num_sols;i++) 
            //     printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
            //     rad_format(q_sols[i*6+0]), rad_format(q_sols[i*6+1]), rad_format(q_sols[i*6+2]), rad_format(q_sols[i*6+3]), rad_format(q_sols[i*6+4]), rad_format(q_sols[i*6+5]));
            for(int j=0;j<num_sols;j++)
            {
                int current_error = 0;
                for(int k=0;k<6;k++)
                {
                    current_error += squre_error(rad_format(q_sols[j*6+k]), current_pos.data[k]);
                }
                if(current_error < min_squre_error)
                {
                    for(int q=0;q<6;q++)
                    {
                        target_pos.data[q] = rad_format(q_sols[j*6+q]);
                    }
                    min_squre_error = current_error;
                    // std::cout<<"1"<<std::endl;
                }
            }
            for(int w=0;w<6;w++)
            {
                // std::cout<<target_pos.data[w]<<std::endl;
            }
            ur5_pos_msg.arm1 = target_pos.data[0];
            ur5_pos_msg.arm2 = target_pos.data[1];
            ur5_pos_msg.arm3 = target_pos.data[2];
            ur5_pos_msg.arm4 = target_pos.data[3];
            ur5_pos_msg.arm5 = target_pos.data[4];
            ur5_pos_msg.arm6 = target_pos.data[5];
            ur5_pub.publish(ur5_pos_msg);
            ifcommand = false;
        }
        admittance_control(end_force);
        loop_rate.sleep();
    }

    
    return 0;
}
#endif
