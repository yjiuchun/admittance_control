#ifndef _UR_KINEMATICS_H_
#define _UR_KINEMATICS_H_

#include<iostream>
#include<cmath>
#include<Eigen/Dense>
#include <array>

#define Eigen Eigen // 替换一下，防止Eigen库不一样

class Ur_Kinematics
{

public:

    double d[6] = {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823};
    double a[6] = {0.0, 0.0, -0.42500, -0.39225, 0.0, 0.0};
    double alph[6] = {0.0, M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2};

   
    struct inverse_8theta
    {
        double th[6][8];
    };

    double now_theta[6];  // 从编码器读出的当前的关节角度 ！！！

    Eigen::MatrixXd AH(int n, double* theta);
    Eigen::Matrix4d forward(double* theta);
    inverse_8theta inverse(Eigen::Matrix4d desired_pos);
    

private:

   
    double default_theta = 100;                                 // 逆解无解时赋值的角度大小
    
};


/*
	功能：  第N个坐标系的转换矩阵
	输入1:  int n
	输入2:  double *theta
	返回:   MatrixXd 4*4的矩阵
*/
Eigen::MatrixXd Ur_Kinematics::AH(int n, double* theta) 
{
	Eigen::Matrix4d T_a = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_d = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Rzt;
	Eigen::Matrix4d Rxa;
	
	T_a(0, 3) = a[n - 1];			
	T_d(2, 3) = d[n - 1];
	
	Rzt <<  cos(theta[n - 1]), -sin(theta[n - 1]), 0, 0,
			sin(theta[n - 1]),  cos(theta[n - 1]), 0, 0,
							0,                 0,  1, 0,
							0, 				  0,  0, 1;
	
	Rxa <<  1,                0,                 0, 0,
			0, cos(alph[n - 1]), -sin(alph[n - 1]), 0,
			0, sin(alph[n - 1]),  cos(alph[n - 1]), 0,
			0,                0,                 0, 1;

	return Rxa * T_a * Rzt * T_d;
}

/*
	功能： 正向运动学，计算末端坐标系相对大地坐标系的转换矩阵
	输入:  double *theta
	返回:   MatrixXd 4*4的矩阵
*/
Eigen::Matrix4d Ur_Kinematics::forward(double* theta)
{
	Eigen::Matrix4d A_1, A_2, A_3, A_4, A_5, A_6, T_06;

	*(theta+1) += M_PI/2;
	*(theta+3) += M_PI/2;

	A_1  = AH(1, theta);
	A_2  = AH(2, theta);
	A_3  = AH(3, theta);
	A_4  = AH(4, theta);
	A_5  = AH(5, theta);
	A_6  = AH(6, theta);

	T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6;

	*(theta+1) -= M_PI/2;
	*(theta+3) -= M_PI/2;

	return T_06;
}

/*
	功能： 逆向运动学，计算各个关节的角度
	输入:  Eigen::Matrix4d desired_pos
	返回:  inverse_8theta 6*8的矩阵
*/
Ur_Kinematics::inverse_8theta Ur_Kinematics::inverse(Eigen::Matrix4d desired_pos)
{
	inverse_8theta theta8;

	

	double d1, d4, d5, a2, a3, nx, ny, ox, oy, ax, ay, az, px, py, pz;
	double forjudgment;
	double epsilon = 0.00001;
	double s5_1,s5_2,s5_3,s5_4;
	double sin234 ,cos234 ,theta234, B2 , B1 ,C, B, A, status;
	double theta23;

	double theta1_1 = default_theta;
	double theta1_2 = default_theta;

	double theta5_1 = default_theta;
	double theta5_2 = default_theta;
	double theta5_3 = default_theta;
	double theta5_4 = default_theta;

	double theta6_1 = default_theta;
	double theta6_2 = default_theta;
	double theta6_3 = default_theta;
	double theta6_4 = default_theta;

	double theta2_1 = default_theta;
	double theta2_2 = default_theta;
	double theta2_3 = default_theta;
	double theta2_4 = default_theta;
	double theta2_5 = default_theta;
	double theta2_6 = default_theta;
	double theta2_7 = default_theta;
	double theta2_8 = default_theta;

	double theta3_1 = default_theta;
	double theta3_2 = default_theta;
	double theta3_3 = default_theta;
	double theta3_4 = default_theta;
	double theta3_5 = default_theta;
	double theta3_6 = default_theta;
	double theta3_7 = default_theta;
	double theta3_8 = default_theta;

	double theta4_1 = default_theta;
	double theta4_2 = default_theta;
	double theta4_3 = default_theta;
	double theta4_4 = default_theta;
	double theta4_5 = default_theta;
	double theta4_6 = default_theta;
	double theta4_7 = default_theta;
	double theta4_8 = default_theta;


	Eigen::Matrix4d R07, R07_inverse;

	

	R07 <<  1, 0, 0,    0,
			0, 1, 0,    0,
			0, 0, 1, d[5],
			0, 0, 0,    1;

	R07_inverse = R07.inverse();

	desired_pos = desired_pos * R07_inverse;

	d1 = d[0]; d4 = d[3]; d5 = d[4]; a2 = a[2]; a3 = a[3];
	nx = desired_pos(0,0); ny = desired_pos(1,0);
	ox = desired_pos(0,1); oy = desired_pos(1,1);
	ax = desired_pos(0,2); ay = desired_pos(1,2); az = desired_pos(2,2);
	px = desired_pos(0,3); py = desired_pos(1,3); pz = desired_pos(2,3);

	// Initialize other theta values similarly

	forjudgment = px * px + py * py - d4 * d4;

	if (forjudgment >= 0)
	{
		theta1_1 = atan2(py, px) - atan2(-d4, sqrt(forjudgment));

		theta1_2 = atan2(py, px) - atan2(-d4, -sqrt(forjudgment));


		s5_1 = sqrt( (-(sin(theta1_1)) * nx + (cos(theta1_1)) * ny)*(-(sin(theta1_1)) * nx + (cos(theta1_1)) * ny)
				+ (-(sin(theta1_1)) * ox + (cos(theta1_1)) * oy)*(-(sin(theta1_1)) * ox + (cos(theta1_1)) * oy) );

		s5_2 = -sqrt( (-(sin(theta1_1)) * nx + (cos(theta1_1)) * ny)*(-(sin(theta1_1)) * nx + (cos(theta1_1)) * ny)
				+ (-(sin(theta1_1)) * ox + (cos(theta1_1)) * oy)*(-(sin(theta1_1)) * ox + (cos(theta1_1)) * oy)   );

		s5_3 = sqrt( (-(sin(theta1_2)) * nx + (cos(theta1_2)) * ny)*(-(sin(theta1_2)) * nx + (cos(theta1_2)) * ny)
				+ (-(sin(theta1_2)) * ox + (cos(theta1_2)) * oy)*(-(sin(theta1_2)) * ox + (cos(theta1_2)) * oy)   );

		s5_4 = -sqrt(  (-(sin(theta1_2)) * nx + (cos(theta1_2)) * ny)*(-(sin(theta1_2)) * nx + (cos(theta1_2)) * ny)
				+ (-(sin(theta1_2)) * ox + (cos(theta1_2)) * oy)*(-(sin(theta1_2)) * ox + (cos(theta1_2)) * oy)   );

		// Calculate theta5_1, theta5_2, theta5_3, theta5_4 similarly

		theta5_1 = atan2(s5_1, ((sin(theta1_1))*ax-(cos(theta1_1))*ay));
		theta5_2 = atan2(s5_2, ((sin(theta1_1))*ax-(cos(theta1_1))*ay));
		theta5_3 = atan2(s5_3, ((sin(theta1_2))*ax-(cos(theta1_2))*ay));
		theta5_4 = atan2(s5_4, ((sin(theta1_2))*ax-(cos(theta1_2))*ay));

		if (s5_1 < -epsilon || s5_1 > epsilon)
		{
			theta6_1 = atan2( (-(sin(theta1_1)) * ox + (cos(theta1_1)) * oy) / s5_1, ((sin(theta1_1)) * nx - (cos(theta1_1)) * ny) / s5_1);

			sin234 = -az / s5_1;
			cos234 = -((cos(theta1_1)) * ax + (sin(theta1_1)) * ay) / s5_1;
			theta234 = atan2(sin234, cos234);
			B2 = pz - d1 + d5 * cos234;
			B1 = (cos(theta1_1)) * px + (sin(theta1_1)) * py - d5 * sin234;
			C = B1*B1 + B2*B2 + a2*a2 - a3*a3;
			B = 2 * B1 * a2;
			A = -2 * B2 * a2;
			status = A*A + B*B - C*C;

			if (status >= epsilon)
			{
				theta2_1 = atan2(B, A) - atan2(C, sqrt(status));
				theta2_2 = atan2(B, A) - atan2(C, -sqrt(status));

				theta23 = atan2((B2 - a2 * (sin(theta2_1))) / a3, (B1 - a2 * (cos(theta2_1))) / a3);
				theta3_1 = theta23 - theta2_1;
				theta4_1 = theta234 - theta23;

				theta23 = atan2((B2 - a2 * (sin(theta2_2))) / a3, (B1 - a2 * (cos(theta2_2))) / a3);
				theta3_2 = theta23 - theta2_2;
				theta4_2 = theta234 - theta23;

			}
			

		}
		
		//----------------------------2-------------------------------------

		if (s5_2 < -epsilon || s5_2 > epsilon)
		{
			theta6_2 = atan2( (-(sin(theta1_1)) * ox + (cos(theta1_1)) * oy) / s5_2, ((sin(theta1_1)) * nx - (cos(theta1_1)) * ny) / s5_2);
			sin234 = -az / s5_2;
			cos234 = -((cos(theta1_1)) * ax + (sin(theta1_1)) * ay) / s5_2;
			theta234 = atan2(sin234, cos234);
			B2 = pz - d1 + d5 * cos234;
			B1 = (cos(theta1_1)) * px + (sin(theta1_1)) * py - d5 * sin234;
			C = B1*B1 + B2*B2 + a2*a2 - a3*a3;
			B = 2 * B1 * a2;
			A = -2 * B2 * a2;
			status = A*A + B*B - C*C;

			if (status >= epsilon)
			{
				theta2_3 = atan2(B, A) - atan2(C, sqrt(status));
				theta2_4 = atan2(B, A) - atan2(C, -sqrt(status));

				theta23 = atan2((B2 - a2 * (sin(theta2_3))) / a3, (B1 - a2 * (cos(theta2_3))) / a3);
				theta3_3 = theta23 - theta2_3;
				theta4_3 = theta234 - theta23;

				theta23 = atan2((B2 - a2 * (sin(theta2_4))) / a3, (B1 - a2 * (cos(theta2_4))) / a3);
				theta3_4 = theta23 - theta2_4;
				theta4_4 = theta234 - theta23;

			}
			

		}
		
		//--------------------------------3---------------------------

		if (s5_3 < -epsilon || s5_3 > epsilon)
		{
			theta6_3 = atan2((-(sin(theta1_2)) * ox + (cos(theta1_2)) * oy) / s5_3, ((sin(theta1_2)) * nx - (cos(theta1_2)) * ny) / s5_3);
			sin234 = -az / s5_3;
			cos234 = -((cos(theta1_2)) * ax + (sin(theta1_2)) * ay) / s5_3;
			theta234 = atan2(sin234, cos234);
			B2 = pz - d1 + d5 * cos234;
			B1 = (cos(theta1_2)) * px + (sin(theta1_2)) * py - d5 * sin234;
			C = B1*B1 + B2*B2 + a2*a2 - a3*a3;
			B = 2 * B1 * a2;
			A = -2 * B2 * a2;
			status = A*A + B*B - C*C;

			if (status >= epsilon)
			{
				theta2_5 = atan2(B, A) - atan2(C, sqrt(status));
				theta2_6 = atan2(B, A) - atan2(C, -sqrt(status));

				theta23 = atan2((B2 - a2 * (sin(theta2_5))) / a3, (B1 - a2 * (cos(theta2_5))) / a3);
				theta3_5 = theta23 - theta2_5;
				theta4_5 = theta234 - theta23;

				theta23 = atan2((B2 - a2 * (sin(theta2_6))) / a3, (B1 - a2 * (cos(theta2_6))) / a3);
				theta3_6 = theta23 - theta2_6;
				theta4_6 = theta234 - theta23;

			}
			
		}
		
		//------------------------------4---------------------------------------
		if (s5_4 < -epsilon || s5_4 > epsilon)
		{
			theta6_4 = atan2((-(sin(theta1_2)) * ox + (cos(theta1_2)) * oy) / s5_4, ((sin(theta1_2)) * nx - (cos(theta1_2)) * ny) / s5_4);
			sin234 = -az / s5_4;
			cos234 = -((cos(theta1_2)) * ax + (sin(theta1_2)) * ay) / s5_4;
			theta234 = atan2(sin234, cos234);
			B2 = pz - d1 + d5 * cos234;
			B1 = (cos(theta1_2)) * px + (sin(theta1_2)) * py - d5 * sin234;
			C = B1*B1 + B2*B2 + a2*a2 - a3*a3;
			B = 2 * B1 * a2;
			A = -2 * B2 * a2;
			status = A*A + B*B - C*C;

			if (status >= epsilon)
			{
				theta2_7 = atan2(B, A) - atan2(C, sqrt(status));
				theta2_8 = atan2(B, A) - atan2(C, -sqrt(status));

				theta23 = atan2((B2 - a2 * (sin(theta2_7))) / a3, (B1 - a2 * (cos(theta2_7))) / a3);
				theta3_7 = theta23 - theta2_7;
				theta4_7 = theta234 - theta23;

				theta23 = atan2((B2 - a2 * (sin(theta2_8))) / a3, (B1 - a2 * (cos(theta2_8))) / a3);
				theta3_8 = theta23 - theta2_8;
				theta4_8 = theta234 - theta23;

			}
			

		}
		

	}

	

	theta8.th[0][0] = theta1_1;
	theta8.th[1][0] = theta2_1 - M_PI/2;
	theta8.th[2][0] = theta3_1;
	theta8.th[3][0] = theta4_1 - M_PI/2;
	theta8.th[4][0] = theta5_1;
	theta8.th[5][0] = theta6_1;

	theta8.th[0][1] = theta1_1;
	theta8.th[1][1] = theta2_2 - M_PI/2;
	theta8.th[2][1] = theta3_2;
	theta8.th[3][1] = theta4_2 - M_PI/2;
	theta8.th[4][1] = theta5_1;
	theta8.th[5][1] = theta6_1;

	theta8.th[0][2] = theta1_1;
	theta8.th[1][2] = theta2_3 - M_PI/2;
	theta8.th[2][2] = theta3_3;
	theta8.th[3][2] = theta4_3 - M_PI/2;
	theta8.th[4][2] = theta5_2;
	theta8.th[5][2] = theta6_2;

	theta8.th[0][3] = theta1_1;
	theta8.th[1][3] = theta2_4 - M_PI/2;
	theta8.th[2][3] = theta3_4;
	theta8.th[3][3] = theta4_4 - M_PI/2;
	theta8.th[4][3] = theta5_2;
	theta8.th[5][3] = theta6_2;

	theta8.th[0][4] = theta1_2;
	theta8.th[1][4] = theta2_5 - M_PI/2;
	theta8.th[2][4] = theta3_5;
	theta8.th[3][4] = theta4_5 - M_PI/2;
	theta8.th[4][4] = theta5_3;
	theta8.th[5][4] = theta6_3;

	theta8.th[0][5] = theta1_2;
	theta8.th[1][5] = theta2_6 - M_PI/2;
	theta8.th[2][5] = theta3_6;
	theta8.th[3][5] = theta4_6 - M_PI/2;
	theta8.th[4][5] = theta5_3;
	theta8.th[5][5] = theta6_3;

	theta8.th[0][6] = theta1_2;
	theta8.th[1][6] = theta2_7 - M_PI/2;
	theta8.th[2][6] = theta3_7;
	theta8.th[3][6] = theta4_7 - M_PI/2;
	theta8.th[4][6] = theta5_4;
	theta8.th[5][6] = theta6_4;

	theta8.th[0][7] = theta1_2;
	theta8.th[1][7] = theta2_8 - M_PI/2;
	theta8.th[2][7] = theta3_8;
	theta8.th[3][7] = theta4_8 - M_PI/2;
	theta8.th[4][7] = theta5_4;
	theta8.th[5][7] = theta6_4;

	for (int i=0;i<6;i++)
	{
		for (int j=0;j<8;j++)
		{

			if ( theta8.th[i][j] < default_theta )
			{
				theta8.th[i][j] = fmod( theta8.th[i][j] , 2*M_PI );
				if ( theta8.th[i][j] < 0 )
				{
					if ( theta8.th[i][j] < -M_PI )
					{
						theta8.th[i][j] += 2*M_PI;
					}
				}
				else
				{
					if ( theta8.th[i][j] > M_PI )
					{
						theta8.th[i][j] -= 2*M_PI;
					}
				}

			}
		}
	}

	

	return theta8;
}



          


#endif