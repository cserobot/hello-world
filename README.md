# hello-world
just another repository
#pragma once
#include "armadillo"
//-----------------------------------------------------------------------------------------
//	Kinematics and dynamics of a 6 DOF robot, as a research platform in Robot LAB of UJN
//  No shared without permission of authour.
//  Oct 30, 2015
//	CopyRight (c) JC
//-----------------------------------------------------------------------------------------
#define PI 3.1415926
using namespace arma;

class R6KiDy
{
public:
	R6KiDy();
	virtual ~R6KiDy();

public:
	mat DH;	// D-H parameters
	mat The;		// 关节角度
	mat The_k;	// 关节角度历史数据
	mat EndPos;			// Ending pose of the hand
	mat Angel;
	mat Jaco;            //雅可比矩阵
private:
	mat T1;
	mat T2;
	mat T3;
	mat T4;
	mat T5;
	mat T6;
	mat Jm;



public:
	mat getEndPose();		
	mat getJointThe(mat);

	bool posi_analysis(const mat &the, mat &targ); // 正向运动学计算
	bool nega_analysis(mat &the, const mat &targ);// 反向运动学计算
	bool jacoMatrix(const mat &the, mat &Jaco);//雅可比矩阵计算

	void updateTi(const mat &the);		// update Ti(i=1,2,...6) with The


};

