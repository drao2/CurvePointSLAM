#ifndef KALMANCLASS_H
#define KALMANCLASS_H

#include <stdlib.h>
#include <stdio.h>
//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>
#include <string>
#include <iostream>
#include "sys/time.h"
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>

#include "common.h"
/*
#define MEAS_COV 5.0 //Innovation

#define VX_COV 0.5
#define VY_COV 0.05
#define VZ_COV 0.1
#define WX_COV 0.1
#define WY_COV 0.1
#define WZ_COV 0.02
#define PHI_MEAS_COV 0.2
#define THETA_MEAS_COV 0.2
#define Z_MEAS_COV 0.4
*/

#define MEAS_COV 100.0 //Innovation for curve cp meas (in m)
#define PT_MEAS_COV 100.0 //Innovation for pt meas (in pixels)

#define VX_COV 0.05
#define VY_COV 0.05
#define VZ_COV 0.5
#define WX_COV 0.1
#define WY_COV 0.1
#define WZ_COV 0.002
#define PHI_MEAS_COV 0.1
#define THETA_MEAS_COV 0.1
#define Z_MEAS_COV 0.2

#define JAC_EPS 1e-4    //Epsilon for numerical Jacobian

#define DT  0.1

#define VREAL 0.0

#define ROBOT_STATE_SIZE  12

using namespace std;

class KalmanFilter
{
public:
	KalmanFilter();
	~KalmanFilter();
        void AddFirstStates(CvMat * measurement);
	void InitOOP(CvMat *);
	void PredictKF();
	void PredictKF(CvMat * R_predict, CvMat * T_predict);
	void AddNewCurve(CvMat * measurement, CvMat * A);
	void AddNewPoints(double * measurements, int n_pts);
        void UpdateNCurvesAndPoints(CvMat * z, int n, std::vector<CvMat *> * A, vector<int> * curve_num, double * point_meas, int * point_nums, int n_pts);
        void UpdatePoints(double * point_meas, int * point_nums, int n_pts);
        void UpdateOOP(CvMat * z);
        bool CheckValidMeasurement(double phi, double theta, double z, int frames_since_good_measurement);
	void GetPredictedMeasurement(CvMat * z_hat, CvMat * x_current, CvMat * A, CvMat * B, int num_curve1, int num_curve2);
	void GetPredictedMeasurement(CvMat * z_hat, CvMat * x_current, CvMat * A, int num_curve);
	void GetSplitMatrices(double t, CvMat * A1, CvMat * A2);
        CvMat * newMatrix(int rows, int cols, int type);
        CvMat * getState();
        CvMat * getOOPState();
	float getTime();
	void resetTime();
        void get_Reb_derivs(double phi, double theta, double psi, CvMat * R_eb_phi, CvMat * R_eb_theta, CvMat * R_eb_psi);
        void get_Rbe_derivs(double phi, double theta, double psi, CvMat * R_be_phi, CvMat * R_be_theta, CvMat * R_be_psi);
        void printMatrix(CvMat *);
        
        void getHNumeric(CvMat * H,CvMat * x, int n, std::vector<CvMat *> * A, vector<int> * curve_num, int * point_nums, int n_pts);
        
        void getGxNumeric(CvMat * Gx,double * measurement,CvMat * x);
        void getGzNumeric(CvMat * Gz,double * measurement,CvMat * x);
        
        void getGxCurveNumeric(CvMat * Gx,CvMat * z,CvMat * x, CvMat * A);
        void getGzCurveNumeric(CvMat * Gz,CvMat * z,CvMat * x, CvMat * A);
        
        void InitCurve(CvMat * curve, CvMat * z, CvMat * x, CvMat * A);
        void InitPoint(CvMat * pt,double * measurement,CvMat * x);
        void predictPointMeas(CvMat * meas, CvMat * x, int point_num);
	CvMat * A1;
	CvMat * A2;
	CvMat * A1l;
	CvMat * A2l;
	CvMat * A1r;
	CvMat * A2r;
	CvMat * B1;
	CvMat * B2;
	CvMat * B1l;
	CvMat * B2l;
	CvMat * B1r;
	CvMat * B2r;
	CvMat * H;
	CvMat * H1;
	CvMat * H1t;
	CvMat * Hinv;
	CvMat * K1;
	CvMat * delP1;
	CvMat * R1;
	CvMat * R_pts;
	CvMat * S1;
	CvMat * z1;
	CvMat * z_hat1;
	CvMat * delx1;
	CvMat * H2;
	CvMat * H2t;
	CvMat * K2;
	CvMat * delP2;
	CvMat * z2;
	CvMat * delx2;
        CvMat * temp19;
        CvMat * temp16;
        CvMat * temp161;
        CvMat * temp1916;
        CvMat * temp11;
        CvMat * temp118;
        CvMat * temp8;
        CvMat * temp81;
        CvMat * temp4;
        CvMat * temp41;
        CvMat * Reb;
        CvMat * Rebt;
        CvMat * Reb_phi;
        CvMat * Reb_theta;
        CvMat * Reb_psi;
        CvMat * E;
        CvMat * Et;
        CvMat * E_phi;
        CvMat * E_theta;
        CvMat * Rot;
        CvMat * Rotderiv;
        CvMat * x;
        CvMat * xOOP;
        CvMat * P;
        CvMat * POOP;
	CvMat * QOOP;
	CvMat * ROOP;
        CvMat * xcurrent8;
        CvMat * Pcurrent8;

	CvMat * Fpose;
	CvMat * Qpose;
        CvMat * temp31;
        CvMat * temp312;
        CvMat * temp33;
        CvMat * Vcov;
        CvMat * Wcov;

        CvMat * temp38;
        CvMat * temp83;
        CvMat * temp88;
        CvMat * Prr;


        CvMat * Ainv;
        CvMat * Ainvzx;
        CvMat * Ainvzy;
        CvMat * zx;
        CvMat * zy;

        CvMat * Gx;
        CvMat * Gz;


        int num_curves;
        int num_points;
        int num_curves_before_add;
        int num_points_before_add;
        

	timeval start, stop;
	float elapsedTime;

        double last_position[3];
};

	
#endif
