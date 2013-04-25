#include <stdlib.h>
#include <stdio.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <sys/time.h>
#include "displayClass.h"
#include "common.h"
#include "curveFittingClass.h"
#include "kalmanClass.h"
#include </usr/include/gsl/gsl_poly.h>

// 2.31	-1.20	3.03	-1.17	3.76	-1.14	4.48	-1.11


#define LM_OPTS_SZ    	 5 /* max(4, 5) */
#define LM_INFO_SZ    	 10
#define LM_ERROR         -1
#define LM_INIT_MU    	 1E-03
#define LM_STOP_THRESH	 1E-17
#define LM_DIFF_DELTA    1E-06
#define LM_VERSION       "2.5 (December 2009)"


double OOPpose[3];

struct mydata funcdata;



//static int bcoeff[CURVE_ORDER+1][CURVE_ORDER+1];
static timeval start, stop;
static float elapsedTime;
static gsl_poly_complex_workspace * w[6];


    CvMat * xtemp31;
    CvMat * xtemp33;

CurveFittingClass::CurveFittingClass()
{
    for (int i = 0; i < 6; i++)
        w[i] = gsl_poly_complex_workspace_alloc (i+1);
    funcdata.orig_meas = NULL;

    xtemp31 = cvCreateMat(3,1,CV_64FC1);
    xtemp33 = cvCreateMat(3,3,CV_64FC1);


    start_pt1 = cvCreateMat(3,1,CV_64FC1);
    start_pt2 = cvCreateMat(3,1,CV_64FC1);
    mid_pt1 = cvCreateMat(3,1,CV_64FC1);
    mid_pt2 = cvCreateMat(3,1,CV_64FC1);
    end_pt1 = cvCreateMat(3,1,CV_64FC1);
    end_pt2 = cvCreateMat(3,1,CV_64FC1);
    centroid = cvCreateMat(3,1,CV_64FC1);
    Rot1 = cvCreateMat(3,1,CV_64FC1);
    Rot2 = cvCreateMat(3,1,CV_64FC1);
    Rot3 = cvCreateMat(3,1,CV_64FC1);


    M = cvCreateMat(4,3, CV_64FC1);
    W = cvCreateMat(3,1, CV_64FC1);
    U = cvCreateMat(4,3, CV_64FC1);
    V = cvCreateMat(3,3, CV_64FC1);
    normal = cvCreateMat(3,1,CV_64FC1);
    origin = cvCreateMat(3,1,CV_64FC1);
    Rot = cvCreateMat(3,3,CV_64FC1);




}
CurveFittingClass::~CurveFittingClass()
{
    
    for (int i = 0; i < 6; i++)
         gsl_poly_complex_workspace_free (w[i]);
}

/* model to be fitted to measurements: 'second set of features' = f( 'first set' ) */
void modelfunc(double *cp, double *x, int m, int n, void *datax)
{
    struct mydata *data = (struct mydata *)datax;

    double p_left[4*(CURVE_ORDER+1)], p_right[4*(CURVE_ORDER+1)];

    double euler[] = {0.0,cp[4*(CURVE_ORDER+1)],cp[4*(CURVE_ORDER+1)+1]};
    double translation[] = {0.0,0.0,cp[4*(CURVE_ORDER+1)+2]};

    cp_earth2image(cp,p_left,p_right, euler, translation);
    cp_earth2image(&(cp[2*(CURVE_ORDER+1)]),&(p_left[2*(CURVE_ORDER+1)]),&(p_right[2*(CURVE_ORDER+1)]), euler, translation);

    // For each ORIGINAL measurement, figure out the nearest point in Bezier cubic (using current iteration of params)
    // This is the estimated measurement, and the LS algorithm will minimize this distance
    double dist = 0.0;

    //Left image curve 1
    for (int i = 0; i < data->num_left1; i++)
    {
        dist = closest_bezier_pt(p_left,data->t[i],&(data->orig_meas[2*i]), &(x[2*i]));
    }
    //Left image curve 2
    for (int i = data->num_left1; i < (data->num_left1+data->num_left2); i++)
    {
        dist = closest_bezier_pt(&(p_left[2*(CURVE_ORDER+1)]),data->t[i],&(data->orig_meas[2*i]), &(x[2*i]));
    }
    //Right image curve 1
    for (int i = (data->num_left1+data->num_left2); i < (data->num_left1+data->num_left2+data->num_right1); i++)
    {
        dist = closest_bezier_pt(p_right,data->t[i],&(data->orig_meas[2*i]), &(x[2*i]));
    }
    //Right image curve 2
    for (int i = (data->num_left1+data->num_left2+data->num_right1); i < n/2; i++)
    {
        dist = closest_bezier_pt(&(p_right[2*(CURVE_ORDER+1)]),data->t[i],&(data->orig_meas[2*i]), &(x[2*i]));
    }

}

/* Jacobian of expfunc() */
void jacmodelfunc(double *p, double *jac, int m, int n, void *datax)
{
    struct mydata *data = (struct mydata *)datax;

    double theta = p[16];
    double phi = p[17];
    double euler[] = {0.0,p[16],p[17]};
    double translation[] = {0.0,0.0,p[18]};

    double t,xe,xb,ye,yb,zb;
    CvMat * R_be = cvCreateMat(3,3,CV_64FC1);
    generate_Rbe(p[17], p[16], 0.0, R_be);
    double r11 = R_be->data.db[0];
    double r12 = R_be->data.db[1];
    double r13 = R_be->data.db[2];
    double r21 = R_be->data.db[3];
    double r22 = R_be->data.db[4];
    double r23 = R_be->data.db[5];
    double r31 = R_be->data.db[6];
    double r32 = R_be->data.db[7];
    double r33 = R_be->data.db[8];
    double pt_e[3];
    double pt_b[3];

    double xbtheta, xbphi, ybtheta, ybphi, zbtheta, zbphi;


    //LEFT IMAGE, LEFT CURVE
    for (int i = 0; i < data->num_left1; i++)
    {
        t = data->t[i];
        xe = p[0]*pow(t,3.0)+p[1]*pow(t,2.0)+p[2]*t+p[3];
        ye = p[4]*pow(t,3.0)+p[5]*pow(t,2.0)+p[6]*t+p[7];
        pt_e[0] = xe;
        pt_e[1] = ye;
        pt_e[2] = 0.0;

        earth2body(pt_e,pt_b,euler,translation);
        xb = pt_b[0];
        double xbsq = pow(xb,2.0);
        yb = pt_b[1];
        zb = pt_b[2];

        xbtheta = -sin(theta)*xe+cos(theta)*p[18];
        xbphi = 0.0;
        ybtheta = cos(theta)*sin(phi)*xe+sin(theta)*sin(phi)*p[18];
        ybphi = sin(theta)*cos(phi)*xe-sin(phi)*ye-cos(phi)*cos(theta)*p[18];
        zbtheta = cos(theta)*cos(phi)*xe+sin(theta)*cos(phi)*p[18];
        zbphi = -sin(theta)*sin(phi)*xe-cos(phi)*ye+cos(theta)*sin(phi)*p[18];

        jac[2*i*m] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq*pow(t,3.0);//dx/da0
        jac[2*i*m+1] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq*pow(t,2.0);//dx/da1
        jac[2*i*m+2] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq*t;//dx/da2
        jac[2*i*m+3] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq;//dx/da3
        jac[2*i*m+4] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq*pow(t,3.0);//dx/db0
        jac[2*i*m+5] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq*pow(t,2.0);//dx/db1
        jac[2*i*m+6] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq*t;//dx/db2
        jac[2*i*m+7] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq;//dx/db3
        jac[2*i*m+8] = 0.0;//dx/dc0
        jac[2*i*m+9] = 0.0;//dx/dc1
        jac[2*i*m+10] = 0.0;//dx/dc2
        jac[2*i*m+11] = 0.0;//dx/dc3
        jac[2*i*m+12] = 0.0;//dx/dd0
        jac[2*i*m+13] = 0.0;//dx/dd1
        jac[2*i*m+14] = 0.0;//dx/dd2
        jac[2*i*m+15] = 0.0;//dx/dd3
        jac[2*i*m+16] = FX/xbsq*(ybtheta*xb-xbtheta*(yb+0.5*BASELINE));//dx/dtheta
        jac[2*i*m+17] = FX/xbsq*(ybphi*xb-xbphi*(yb+0.5*BASELINE));//dx/dphi
        jac[2*i*m+18] = FX/xbsq*(-r23*xb+r13*(yb+0.5*BASELINE));//dx/dT3

        jac[(2*i+1)*m] = FY*(r31*xb-r11*zb)/xbsq*pow(t,3.0);//dy/da0
        jac[(2*i+1)*m+1] = FY*(r31*xb-r11*zb)/xbsq*pow(t,2.0);//dy/da1
        jac[(2*i+1)*m+2] = FY*(r31*xb-r11*zb)/xbsq*t;//dy/da2
        jac[(2*i+1)*m+3] = FY*(r31*xb-r11*zb)/xbsq;//dy/da3
        jac[(2*i+1)*m+4] = FY*(r32*xb-r12*zb)/xbsq*pow(t,3.0);//dy/db0
        jac[(2*i+1)*m+5] = FY*(r32*xb-r12*zb)/xbsq*pow(t,2.0);//dy/db1
        jac[(2*i+1)*m+6] = FY*(r32*xb-r12*zb)/xbsq*t;//dy/db2
        jac[(2*i+1)*m+7] = FY*(r32*xb-r12*zb)/xbsq;//dy/db3
        jac[(2*i+1)*m+8] = 0.0;//dx/dc0
        jac[(2*i+1)*m+9] = 0.0;//dx/dc1
        jac[(2*i+1)*m+10] = 0.0;//dx/dc2
        jac[(2*i+1)*m+11] = 0.0;//dx/dc3
        jac[(2*i+1)*m+12] = 0.0;//dx/dd0
        jac[(2*i+1)*m+13] = 0.0;//dx/dd1
        jac[(2*i+1)*m+14] = 0.0;//dx/dd2
        jac[(2*i+1)*m+15] = 0.0;//dx/dd3
        jac[(2*i+1)*m+16] = FY/xbsq*(zbtheta*xb-xbtheta*zb);//dy/dtheta
        jac[(2*i+1)*m+17] = FY/xbsq*(zbphi*xb-xbphi*zb);//dy/dphi
        jac[(2*i+1)*m+18] = FY/xbsq*(-r33*xb+r13*zb);//dy/dT3

    }
    //LEFT IMAGE, RIGHT CURVE
    for (int i = data->num_left1; i < (data->num_left1+data->num_left2); i++)
    {
        t = data->t[i];
        xe = p[8]*pow(t,3.0)+p[9]*pow(t,2.0)+p[10]*t+p[11];
        ye = p[12]*pow(t,3.0)+p[13]*pow(t,2.0)+p[14]*t+p[15];
        pt_e[0] = xe;
        pt_e[1] = ye;
        pt_e[2] = 0.0;

        earth2body(pt_e,pt_b,euler,translation);
        xb = pt_b[0];
        double xbsq = pow(xb,2.0);
        yb = pt_b[1];
        zb = pt_b[2];

        xbtheta = -sin(theta)*xe+cos(theta)*p[18];
        xbphi = 0.0;
        ybtheta = cos(theta)*sin(phi)*xe+sin(theta)*sin(phi)*p[18];
        ybphi = sin(theta)*cos(phi)*xe-sin(phi)*ye-cos(phi)*cos(theta)*p[18];
        zbtheta = cos(theta)*cos(phi)*xe+sin(theta)*cos(phi)*p[18];
        zbphi = -sin(theta)*sin(phi)*xe-cos(phi)*ye+cos(theta)*sin(phi)*p[18];

        jac[2*i*m] = 0.0;//dx/da0
        jac[2*i*m+1] = 0.0;//dx/da1
        jac[2*i*m+2] = 0.0;//dx/da2
        jac[2*i*m+3] = 0.0;//dx/da3
        jac[2*i*m+4] = 0.0;//dx/db0
        jac[2*i*m+5] = 0.0;//dx/db1
        jac[2*i*m+6] = 0.0;//dx/db2
        jac[2*i*m+7] = 0.0;//dx/db3
        jac[2*i*m+8] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq*pow(t,3.0);//dx/dc0
        jac[2*i*m+9] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq*pow(t,2.0);//dx/dc1
        jac[2*i*m+10] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq*t;//dx/dc2
        jac[2*i*m+11] = FX*(r21*xb-r11*(yb+0.5*BASELINE))/xbsq;//dx/dc3
        jac[2*i*m+12] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq*pow(t,3.0);//dx/dd0
        jac[2*i*m+13] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq*pow(t,2.0);//dx/dd1
        jac[2*i*m+14] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq*t;//dx/dd2
        jac[2*i*m+15] = FX*(r22*xb-r12*(yb+0.5*BASELINE))/xbsq;//dx/dd3
        jac[2*i*m+16] = FX/xbsq*(ybtheta*xb-xbtheta*(yb+0.5*BASELINE));//dx/dtheta
        jac[2*i*m+17] = FX/xbsq*(ybphi*xb-xbphi*(yb+0.5*BASELINE));//dx/dphi
        jac[2*i*m+18] = FX/xbsq*(-r23*xb+r13*(yb+0.5*BASELINE));//dx/dT3

        jac[(2*i+1)*m] = 0.0;//dx/da0
        jac[(2*i+1)*m+1] = 0.0;//dx/da1
        jac[(2*i+1)*m+2] = 0.0;//dx/da2
        jac[(2*i+1)*m+3] = 0.0;//dx/da3
        jac[(2*i+1)*m+4] = 0.0;//dx/db0
        jac[(2*i+1)*m+5] = 0.0;//dx/db1
        jac[(2*i+1)*m+6] = 0.0;//dx/db2
        jac[(2*i+1)*m+7] = 0.0;//dx/db3
        jac[(2*i+1)*m+8] = FY*(r31*xb-r11*zb)/xbsq*pow(t,3.0);//dy/dc0
        jac[(2*i+1)*m+9] = FY*(r31*xb-r11*zb)/xbsq*pow(t,2.0);//dy/dc1
        jac[(2*i+1)*m+10] = FY*(r31*xb-r11*zb)/xbsq*t;//dy/dc2
        jac[(2*i+1)*m+11] = FY*(r31*xb-r11*zb)/xbsq;//dy/dc3
        jac[(2*i+1)*m+12] = FY*(r32*xb-r12*zb)/xbsq*pow(t,3.0);//dy/dd0
        jac[(2*i+1)*m+13] = FY*(r32*xb-r12*zb)/xbsq*pow(t,2.0);//dy/dd1
        jac[(2*i+1)*m+14] = FY*(r32*xb-r12*zb)/xbsq*t;//dy/dd2
        jac[(2*i+1)*m+15] = FY*(r32*xb-r12*zb)/xbsq;//dy/dd3
        jac[(2*i+1)*m+16] = FY/xbsq*(zbtheta*xb-xbtheta*zb);//dy/dtheta
        jac[(2*i+1)*m+17] = FY/xbsq*(zbphi*xb-xbphi*zb);//dy/dphi
        jac[(2*i+1)*m+18] = FY/xbsq*(-r33*xb+r13*zb);//dy/dT3
    }
    //RIGHT IMAGE, LEFT CURVE
    for (int i = (data->num_left1+data->num_left2); i < (data->num_left1+data->num_left2+data->num_right1); i++)
    {
        t = data->t[i];
        xe = p[0]*pow(t,3.0)+p[1]*pow(t,2.0)+p[2]*t+p[3];
        ye = p[4]*pow(t,3.0)+p[5]*pow(t,2.0)+p[6]*t+p[7];
        pt_e[0] = xe;
        pt_e[1] = ye;
        pt_e[2] = 0.0;

        earth2body(pt_e,pt_b,euler,translation);
        xb = pt_b[0];
        double xbsq = pow(xb,2.0);
        yb = pt_b[1];
        zb = pt_b[2];

        xbtheta = -sin(theta)*xe+cos(theta)*p[18];
        xbphi = 0.0;
        ybtheta = cos(theta)*sin(phi)*xe+sin(theta)*sin(phi)*p[18];
        ybphi = sin(theta)*cos(phi)*xe-sin(phi)*ye-cos(phi)*cos(theta)*p[18];
        zbtheta = cos(theta)*cos(phi)*xe+sin(theta)*cos(phi)*p[18];
        zbphi = -sin(theta)*sin(phi)*xe-cos(phi)*ye+cos(theta)*sin(phi)*p[18];

        jac[2*i*m] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq*pow(t,3.0);//dx/da0
        jac[2*i*m+1] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq*pow(t,2.0);//dx/da1
        jac[2*i*m+2] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq*t;//dx/da2
        jac[2*i*m+3] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq;//dx/da3
        jac[2*i*m+4] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq*pow(t,3.0);//dx/db0
        jac[2*i*m+5] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq*pow(t,2.0);//dx/db1
        jac[2*i*m+6] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq*t;//dx/db2
        jac[2*i*m+7] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq;//dx/db3
        jac[2*i*m+8] = 0.0;//dx/dc0
        jac[2*i*m+9] = 0.0;//dx/dc1
        jac[2*i*m+10] = 0.0;//dx/dc2
        jac[2*i*m+11] = 0.0;//dx/dc3
        jac[2*i*m+12] = 0.0;//dx/dd0
        jac[2*i*m+13] = 0.0;//dx/dd1
        jac[2*i*m+14] = 0.0;//dx/dd2
        jac[2*i*m+15] = 0.0;//dx/dd3
        jac[2*i*m+16] = FX/xbsq*(ybtheta*xb-xbtheta*(yb-0.5*BASELINE));//dx/dtheta
        jac[2*i*m+17] = FX/xbsq*(ybphi*xb-xbphi*(yb-0.5*BASELINE));//dx/dphi
        jac[2*i*m+18] = FX/xbsq*(-r23*xb+r13*(yb-0.5*BASELINE));//dx/dT3

        jac[(2*i+1)*m] = FY*(r31*xb-r11*zb)/xbsq*pow(t,3.0);//dy/da0
        jac[(2*i+1)*m+1] = FY*(r31*xb-r11*zb)/xbsq*pow(t,2.0);//dy/da1
        jac[(2*i+1)*m+2] = FY*(r31*xb-r11*zb)/xbsq*t;//dy/da2
        jac[(2*i+1)*m+3] = FY*(r31*xb-r11*zb)/xbsq;//dy/da3
        jac[(2*i+1)*m+4] = FY*(r32*xb-r12*zb)/xbsq*pow(t,3.0);//dy/db0
        jac[(2*i+1)*m+5] = FY*(r32*xb-r12*zb)/xbsq*pow(t,2.0);//dy/db1
        jac[(2*i+1)*m+6] = FY*(r32*xb-r12*zb)/xbsq*t;//dy/db2
        jac[(2*i+1)*m+7] = FY*(r32*xb-r12*zb)/xbsq;//dy/db3
        jac[(2*i+1)*m+8] = 0.0;//dx/dc0
        jac[(2*i+1)*m+9] = 0.0;//dx/dc1
        jac[(2*i+1)*m+10] = 0.0;//dx/dc2
        jac[(2*i+1)*m+11] = 0.0;//dx/dc3
        jac[(2*i+1)*m+12] = 0.0;//dx/dd0
        jac[(2*i+1)*m+13] = 0.0;//dx/dd1
        jac[(2*i+1)*m+14] = 0.0;//dx/dd2
        jac[(2*i+1)*m+15] = 0.0;//dx/dd3
        jac[(2*i+1)*m+16] = FY/xbsq*(zbtheta*xb-xbtheta*zb);//dy/dtheta
        jac[(2*i+1)*m+17] = FY/xbsq*(zbphi*xb-xbphi*zb);//dy/dphi
        jac[(2*i+1)*m+18] = FY/xbsq*(-r33*xb+r13*zb);//dy/dT3
    }
    //RIGHT IMAGE, RIGHT CURVE
    for (int i = (data->num_left1+data->num_left2+data->num_right1); i < n/2; i++)
    {
        t = data->t[i];
        xe = p[8]*pow(t,3.0)+p[9]*pow(t,2.0)+p[10]*t+p[11];
        ye = p[12]*pow(t,3.0)+p[13]*pow(t,2.0)+p[14]*t+p[15];
        pt_e[0] = xe;
        pt_e[1] = ye;
        pt_e[2] = 0.0;

        earth2body(pt_e,pt_b,euler,translation);
        xb = pt_b[0];
        double xbsq = pow(xb,2.0);
        yb = pt_b[1];
        zb = pt_b[2];

        xbtheta = -sin(theta)*xe+cos(theta)*p[18];
        xbphi = 0.0;
        ybtheta = cos(theta)*sin(phi)*xe+sin(theta)*sin(phi)*p[18];
        ybphi = sin(theta)*cos(phi)*xe-sin(phi)*ye-cos(phi)*cos(theta)*p[18];
        zbtheta = cos(theta)*cos(phi)*xe+sin(theta)*cos(phi)*p[18];
        zbphi = -sin(theta)*sin(phi)*xe-cos(phi)*ye+cos(theta)*sin(phi)*p[18];

        jac[2*i*m] = 0.0;//dx/da0
        jac[2*i*m+1] = 0.0;//dx/da1
        jac[2*i*m+2] = 0.0;//dx/da2
        jac[2*i*m+3] = 0.0;//dx/da3
        jac[2*i*m+4] = 0.0;//dx/db0
        jac[2*i*m+5] = 0.0;//dx/db1
        jac[2*i*m+6] = 0.0;//dx/db2
        jac[2*i*m+7] = 0.0;//dx/db3
        jac[2*i*m+8] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq*pow(t,3.0);//dx/dc0
        jac[2*i*m+9] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq*pow(t,2.0);//dx/dc1
        jac[2*i*m+10] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq*t;//dx/dc2
        jac[2*i*m+11] = FX*(r21*xb-r11*(yb-0.5*BASELINE))/xbsq;//dx/dc3
        jac[2*i*m+12] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq*pow(t,3.0);//dx/dd0
        jac[2*i*m+13] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq*pow(t,2.0);//dx/dd1
        jac[2*i*m+14] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq*t;//dx/dd2
        jac[2*i*m+15] = FX*(r22*xb-r12*(yb-0.5*BASELINE))/xbsq;//dx/dd3
        jac[2*i*m+16] = FX/xbsq*(ybtheta*xb-xbtheta*(yb-0.5*BASELINE));//dx/dtheta
        jac[2*i*m+17] = FX/xbsq*(ybphi*xb-xbphi*(yb-0.5*BASELINE));//dx/dphi
        jac[2*i*m+18] = FX/xbsq*(-r23*xb+r13*(yb-0.5*BASELINE));//dx/dT3

        jac[(2*i+1)*m] = 0.0;//dy/da0
        jac[(2*i+1)*m+1] = 0.0;//dy/da1
        jac[(2*i+1)*m+2] = 0.0;//dy/da2
        jac[(2*i+1)*m+3] = 0.0;//dy/da3
        jac[(2*i+1)*m+4] = 0.0;//dy/db0
        jac[(2*i+1)*m+5] = 0.0;//dy/db1
        jac[(2*i+1)*m+6] = 0.0;//dy/db2
        jac[(2*i+1)*m+7] = 0.0;//dy/db3
        jac[(2*i+1)*m+8] = FY*(r31*xb-r11*zb)/xbsq*pow(t,3.0);//dy/dc0
        jac[(2*i+1)*m+9] = FY*(r31*xb-r11*zb)/xbsq*pow(t,2.0);//dy/dc1
        jac[(2*i+1)*m+10] = FY*(r31*xb-r11*zb)/xbsq*t;//dy/dc2
        jac[(2*i+1)*m+11] = FY*(r31*xb-r11*zb)/xbsq;//dy/dc3
        jac[(2*i+1)*m+12] = FY*(r32*xb-r12*zb)/xbsq*pow(t,3.0);//dy/dd0
        jac[(2*i+1)*m+13] = FY*(r32*xb-r12*zb)/xbsq*pow(t,2.0);//dy/dd1
        jac[(2*i+1)*m+14] = FY*(r32*xb-r12*zb)/xbsq*t;//dy/dd2
        jac[(2*i+1)*m+15] = FY*(r32*xb-r12*zb)/xbsq;//dy/dd3
        jac[(2*i+1)*m+16] = FY/xbsq*(zbtheta*xb-xbtheta*zb);//dy/dtheta
        jac[(2*i+1)*m+17] = FY/xbsq*(zbphi*xb-xbphi*zb);//dy/dphi
        jac[(2*i+1)*m+18] = FY/xbsq*(-r33*xb+r13*zb);//dy/dT3
    }

    cvReleaseMat(&R_be);

}


void CurveFittingClass::fit_curve(double * params, CvMat ** featuresLeftImage, CvMat ** featuresRightImage, DisplayClass * display)
{

        gettimeofday(&start, NULL);
        //19 params passed in: 8 poly coeffs for x and y (in t) for each curve, theta, phi, z

        funcdata.disp = display;        

        //Determine how many Left Image and Right Image points there are for each curve
        int num_pts_left = featuresLeftImage[0]->rows/2+featuresLeftImage[1]->rows/2;
        int num_pts_right = featuresRightImage[0]->rows/2+featuresRightImage[1]->rows/2;
        int num_pts_left1 = featuresLeftImage[0]->rows/2, num_pts_left2 = featuresLeftImage[1]->rows/2, num_pts_right1 = featuresRightImage[0]->rows/2, num_pts_right2 = featuresRightImage[1]->rows/2;
        
        int num_pts = num_pts_left+num_pts_right;

        
        const int num_meas=2*num_pts, m = (4*(CURVE_ORDER+1)+3);
        double final_meas[num_meas], t[num_pts];
        int curve_num[num_pts];

        funcdata.orig_meas = &(final_meas[0]);
        funcdata.num_left1 = num_pts_left1;
        funcdata.num_right1 = num_pts_right1;
        funcdata.num_left2 = num_pts_left2;
        funcdata.num_right2 = num_pts_right2;
        
        // Set up measurements (from features) and the t value for each one (faster approx of nearest point)
        for (int i = 0; i < num_pts_left; i++)
        {
            if (i < num_pts_left1)
            {
                final_meas[2*i] = featuresLeftImage[0]->data.db[2*i]+POINT_OFFSET;
                final_meas[2*i+1] = featuresLeftImage[0]->data.db[2*i+1]+POINT_OFFSET;
            }
            else
            {
                final_meas[2*i] = featuresLeftImage[1]->data.db[2*(i-num_pts_left1)]+POINT_OFFSET;
                final_meas[2*i+1] = featuresLeftImage[1]->data.db[2*(i-num_pts_left1)+1]+POINT_OFFSET;
            }

            curve_num[i] = i < num_pts_left1? 1 : 2;
        }

        double firstx = final_meas[0];
        double firsty = final_meas[1];
        double lastx = final_meas[2*num_pts_left1-2];
        double lasty = final_meas[2*num_pts_left1-1];
        for (int i = 0; i < num_pts_left1; i++)
        {
            t[i] = (final_meas[2*i+1]-firsty)/(lasty-firsty);
            t[i] = ((t[i] < 0.0) ? 0.0 : t[i]);
            t[i] = ((t[i] > 1.0) ? 1.0 : t[i]);
        }
            firstx = final_meas[2*num_pts_left1];
            firsty = final_meas[2*num_pts_left1+1];
            lastx = final_meas[2*num_pts_left-2];
            lasty = final_meas[2*num_pts_left-1];
        for (int i = num_pts_left1; i < num_pts_left; i++)
        {
            t[i] = (final_meas[2*i+1]-firsty)/(lasty-firsty);
            t[i] = ((t[i] < 0.0) ? 0.0 : t[i]);
            t[i] = ((t[i] > 1.0) ? 1.0 : t[i]);
        }

        for (int i = num_pts_left; i < num_pts; i++)
        {
            if (i < num_pts_left+num_pts_right1)
            {
                final_meas[2*i] = featuresRightImage[0]->data.db[2*(i-num_pts_left)]-POINT_OFFSET;
                final_meas[2*i+1] = featuresRightImage[0]->data.db[2*(i-num_pts_left)+1]-POINT_OFFSET;
            }
            else
            {
                final_meas[2*i] = featuresRightImage[1]->data.db[2*(i-num_pts_left-num_pts_right1)]-POINT_OFFSET;
                final_meas[2*i+1] = featuresRightImage[1]->data.db[2*(i-num_pts_left-num_pts_right1)+1]-POINT_OFFSET;
            }

            if (i < num_pts_left+num_pts_right1)
                curve_num[i] = 1;
            else
                curve_num[i] = 2;
        }
            
            firstx = final_meas[2*num_pts_left];
            firsty = final_meas[2*num_pts_left+1];
            lastx = final_meas[2*(num_pts_left+num_pts_right1)-2];
            lasty = final_meas[2*(num_pts_left+num_pts_right1)-1];
        for (int i = num_pts_left; i < num_pts_left+num_pts_right1; i++)
        {
            t[i] = (final_meas[2*i+1]-firsty)/(lasty-firsty);
            t[i] = ((t[i] < 0.0) ? 0.0 : t[i]);
            t[i] = ((t[i] > 1.0) ? 1.0 : t[i]);
        }

            firstx = final_meas[2*(num_pts_left+num_pts_right1)];
            firsty = final_meas[2*(num_pts_left+num_pts_right1)+1];
            lastx = final_meas[2*num_pts-2];
            lasty = final_meas[2*num_pts-1];
        for (int i = num_pts_left+num_pts_right1; i < num_pts; i++)
        {
            t[i] = (final_meas[2*i+1]-firsty)/(lasty-firsty);
            t[i] = ((t[i] < 0.0) ? 0.0 : t[i]);
            t[i] = ((t[i] > 1.0) ? 1.0 : t[i]);
        }

        


        //Initial parameter estimates
        double euler[] = {0.0,params[4*(CURVE_ORDER+1)],params[4*(CURVE_ORDER+1)+1]};
        double translation[] = {0.0,0.0,params[4*(CURVE_ORDER+1)+2]};

        double control_pts1[2*(CURVE_ORDER+1)], control_pts2[2*(CURVE_ORDER+1)];

        start_pt1->data.db[0] = FX*BASELINE/(final_meas[0]-final_meas[2*num_pts_left]);
        start_pt1->data.db[1] = 0.5*(start_pt1->data.db[0]/FX*(final_meas[0]-CX)-0.5*BASELINE)+0.5*(start_pt1->data.db[0]/FX*(final_meas[2*num_pts_left]-CX)+0.5*BASELINE);
        start_pt1->data.db[2] = 0.5*start_pt1->data.db[0]/FY*(final_meas[1]-CY)+0.5*start_pt1->data.db[0]/FY*(final_meas[2*num_pts_left+1]-CY);


        end_pt1->data.db[0] = FX*BASELINE/(final_meas[2*(num_pts_left1-1)]-final_meas[2*(num_pts_left+num_pts_right1-1)]);
        end_pt1->data.db[1] = 0.5*(end_pt1->data.db[0]/FX*(final_meas[2*(num_pts_left1-1)]-CX)-0.5*BASELINE)+0.5*(end_pt1->data.db[0]/FX*(final_meas[2*(num_pts_left+num_pts_right1-1)]-CX)+0.5*BASELINE);
        end_pt1->data.db[2] = 0.5*end_pt1->data.db[0]/FY*(final_meas[2*(num_pts_left1-1)+1]-CY)+0.5*end_pt1->data.db[0]/FY*(final_meas[2*(num_pts_left+num_pts_right1-1)+1]-CY);

        mid_pt1->data.db[0] = FX*BASELINE/(final_meas[2*(num_pts_left1/2-1)]-final_meas[2*(num_pts_left+num_pts_right1/2-1)]);
        mid_pt1->data.db[1] = 0.5*(mid_pt1->data.db[0]/FX*(final_meas[2*(num_pts_left1/2-1)]-CX)-0.5*BASELINE)+0.5*(mid_pt1->data.db[0]/FX*(final_meas[2*(num_pts_left+num_pts_right1/2-1)]-CX)+0.5*BASELINE);
        mid_pt1->data.db[2] = 0.5*mid_pt1->data.db[0]/FY*(final_meas[2*(num_pts_left1/2-1)+1]-CY)+0.5*mid_pt1->data.db[0]/FY*(final_meas[2*(num_pts_left+num_pts_right1/2-1)+1]-CY);


        start_pt2->data.db[0] = FX*BASELINE/(final_meas[2*num_pts_left1]-final_meas[2*(num_pts_left+num_pts_right1)]);
        start_pt2->data.db[1] = 0.5*(start_pt2->data.db[0]/FX*(final_meas[2*num_pts_left1]-CX)-0.5*BASELINE)+0.5*(start_pt2->data.db[0]/FX*(final_meas[2*(num_pts_left+num_pts_right1)]-CX)+0.5*BASELINE);
        start_pt2->data.db[2] = 0.5*start_pt2->data.db[0]/FY*(final_meas[2*num_pts_left1+1]-CY)+0.5*start_pt2->data.db[0]/FY*(final_meas[2*(num_pts_left+num_pts_right1)+1]-CY);


        end_pt2->data.db[0] = FX*BASELINE/(final_meas[2*(num_pts_left-1)]-final_meas[2*(num_pts-1)]);
        end_pt2->data.db[1] = 0.5*(end_pt2->data.db[0]/FX*(final_meas[2*(num_pts_left-1)]-CX)-0.5*BASELINE)+0.5*(end_pt2->data.db[0]/FX*(final_meas[2*(num_pts-1)]-CX)+0.5*BASELINE);
        end_pt2->data.db[2] = 0.5*end_pt2->data.db[0]/FY*(final_meas[2*(num_pts_left-1)+1]-CY)+0.5*end_pt2->data.db[0]/FY*(final_meas[2*(num_pts-1)+1]-CY);

        mid_pt2->data.db[0] = FX*BASELINE/(final_meas[2*(num_pts_left-num_pts_left2/2-1)]-final_meas[2*(num_pts-num_pts_right2/2-1)]);
        mid_pt2->data.db[1] = 0.5*(mid_pt2->data.db[0]/FX*(final_meas[2*(num_pts_left-num_pts_left2/2-1)]-CX)-0.5*BASELINE)+0.5*(mid_pt2->data.db[0]/FX*(final_meas[2*(num_pts-num_pts_right2/2-1)]-CX)+0.5*BASELINE);
        mid_pt2->data.db[2] = 0.5*mid_pt2->data.db[0]/FY*(final_meas[2*(num_pts_left-num_pts_left2/2-1)+1]-CY)+0.5*mid_pt2->data.db[0]/FY*(final_meas[2*(num_pts-num_pts_right2/2-1)+1]-CY);

        centroid->data.db[0] = 0.25*(start_pt1->data.db[0]+start_pt2->data.db[0]+end_pt1->data.db[0]+end_pt2->data.db[0]);
        centroid->data.db[1] = 0.25*(start_pt1->data.db[1]+start_pt2->data.db[1]+end_pt1->data.db[1]+end_pt2->data.db[1]);
        centroid->data.db[2] = 0.25*(start_pt1->data.db[2]+start_pt2->data.db[2]+end_pt1->data.db[2]+end_pt2->data.db[2]);


        //Find the normal vector to the plane they lie in

            for (int j = 0; j < 3; j++)
            {
                cvmSet(M,0,j,start_pt1->data.db[j]-centroid->data.db[j]);
                cvmSet(M,1,j,start_pt2->data.db[j]-centroid->data.db[j]);
                cvmSet(M,2,j,end_pt1->data.db[j]-centroid->data.db[j]);
                cvmSet(M,3,j,end_pt2->data.db[j]-centroid->data.db[j]);
            }
        cvSVD(M, W, U, V, 0);

        normal->data.db[0] = V->data.db[2];
        normal->data.db[1] = V->data.db[5];
        normal->data.db[2] = V->data.db[8];

        double d = cvDotProduct(normal,centroid);
        double theta = -asin(normal->data.db[0]);
        double phi = asin(normal->data.db[1]);

        //Find the point of the 'ground' co-ord axis
        origin->data.db[0] = d*normal->data.db[0];
        origin->data.db[1] = d*normal->data.db[1];
        origin->data.db[2] = d*normal->data.db[2];


        double orignorm = cvNorm(origin,NULL,CV_L2,NULL);
        //ground z-axis
        Rot3->data.db[0] = origin->data.db[0]/orignorm;
        Rot3->data.db[1] = origin->data.db[1]/orignorm;
        Rot3->data.db[2] = origin->data.db[2]/orignorm;
        
        Rot->data.db[0] = cos(theta);
        Rot->data.db[3] = 0.0;
        Rot->data.db[6] = -sin(theta);
        Rot->data.db[1] = sin(theta)*sin(phi);
        Rot->data.db[4] = cos(phi);
        Rot->data.db[7] = cos(theta)*sin(phi);
        Rot->data.db[2] = sin(theta)*cos(phi);
        Rot->data.db[5] = -sin(phi);
        Rot->data.db[8] = cos(theta)*cos(phi);


        cvSub(start_pt1,origin,start_pt1);
        cvMatMul(Rot,start_pt1,start_pt1);
        cvSub(start_pt2,origin,start_pt2);
        cvMatMul(Rot,start_pt2,start_pt2);
        cvSub(mid_pt1,origin,mid_pt1);
        cvMatMul(Rot,mid_pt1,mid_pt1);
        cvSub(mid_pt2,origin,mid_pt2);
        cvMatMul(Rot,mid_pt2,mid_pt2);
        cvSub(end_pt1,origin,end_pt1);
        cvMatMul(Rot,end_pt1,end_pt1);
        cvSub(end_pt2,origin,end_pt2);
        cvMatMul(Rot,end_pt2,end_pt2);
        
        /*cout << "INIT CONDS:\n";
        printMatrix(start_pt1);
        printMatrix(end_pt1);
        printMatrix(start_pt2);
        printMatrix(end_pt2);*/



        control_pts1[0] = start_pt1->data.db[0];
        control_pts1[1] = start_pt1->data.db[1];
        control_pts1[6] = end_pt1->data.db[0];
        control_pts1[7] = end_pt1->data.db[1];
        control_pts2[0] = start_pt2->data.db[0];
        control_pts2[1] = start_pt2->data.db[1];
        control_pts2[6] = end_pt2->data.db[0];
        control_pts2[7] = end_pt2->data.db[1];

        for (int i = 1; i < CURVE_ORDER; i++)
        {
            control_pts1[2*i] = (CURVE_ORDER-i)*control_pts1[0]/CURVE_ORDER + i*control_pts1[2*(CURVE_ORDER)]/CURVE_ORDER;
            control_pts1[2*i+1] = (CURVE_ORDER-i)*control_pts1[1]/CURVE_ORDER + i*control_pts1[2*(CURVE_ORDER)+1]/CURVE_ORDER;
            control_pts2[2*i] = (CURVE_ORDER-i)*control_pts2[0]/CURVE_ORDER + i*control_pts2[2*(CURVE_ORDER)]/CURVE_ORDER;
            control_pts2[2*i+1] = (CURVE_ORDER-i)*control_pts2[1]/CURVE_ORDER + i*control_pts2[2*(CURVE_ORDER)+1]/CURVE_ORDER;
        }



        for (int i = 0; i < 2*(CURVE_ORDER+1); i++)
            params[i] = control_pts1[i];
        for (int i = 0; i < 2*(CURVE_ORDER+1); i++)
            params[i+2*(CURVE_ORDER+1)] = control_pts2[i];

        params[16] = theta;
        params[17] = phi;
        params[18] = -d;

        OOPpose[0] = theta;
        OOPpose[1] = phi;
        OOPpose[2] = -d;


        funcdata.t = &(t[0]);
        funcdata.curve_num = &(curve_num[0]);



        

    double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    int ret;
    
    /* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */
    opts[0]=1E-10; opts[1]=1E-10; opts[2]=1E-10; opts[3]=1E-20;
    opts[4]=LM_DIFF_DELTA*1E1; // relevant only if the finite difference Jacobian version is used


    double lb[m], ub[m];
    for (int i = 0; i < 16; i++)
    {
        lb[i] = -DBL_MAX;
        ub[i] = DBL_MAX;
    }
    lb[16] = -0.05;
    ub[16] = 0.05;
    lb[17] = -0.05;
    ub[17] = 0.05;
    lb[18] = -1.05;
    ub[18] = -0.95;

    /* invoke the optimization function */
    //ret=dlevmar_bc_dif(modelfunc, params, final_meas, m, num_meas, lb, ub, 50, opts, info, NULL, NULL, (void *) &funcdata); // without Jacobian
    //ret=dlevmar_bc_der(modelfunc, jacmodelfunc, params, final_meas, m, num_meas, lb, ub, 50, opts, info, NULL, NULL, (void *) &funcdata); // with analytic Jacobian
    ret=dlevmar_dif(modelfunc, params, final_meas, m, num_meas, 1000, opts, info, NULL, NULL, (void *) &funcdata); // without Jacobian //ret=dlevmar_bc_dif(modelfunc, params, final_meas, m, num_meas, lb, ub, 1000, opts, info, NULL, NULL, (void *) &funcdata); // without Jacobian
    //ret=dlevmar_der(modelfunc, jacmodelfunc, params, final_meas, m, num_meas, 50, opts, info, NULL, NULL, (void *) &funcdata); // with analytic Jacobian
    fitting_error = info[1];
    if (params[18] > 0.0)
    {
        for (int i = 0; i < 8; i++)
        {
            params[2*i+1] *= -1.0;
        }
        params[16] *= -1.0;
        params[17] += PI;
        params[18] *= -1.0;
    }
    if (params[16] < -PI/2.0)
    {
        for (int i = 0; i < 8; i++)
        {
            params[2*i+1] *= -1.0;
        }
        params[17] = (-PI - params[17]);
        params[18] += PI;
    }
    
    while(params[16] > PI)
        params[16] -= 2*PI;
    while(params[16] < -PI)
        params[16] += 2*PI;
    while(params[17] > PI)
        params[17] -= 2*PI;
    while(params[17] < -PI)
        params[17] += 2*PI;
    
    
    //printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
    
        //printf("Best fit parameters:\t");
        //for (int i = 0; i < 19; i++)
        //    printf("%.2f ", params[i]);
        //cout << endl;
    gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

}


float CurveFittingClass::getTime()
{
    return elapsedTime;
}

void CurveFittingClass::resetTime()
{
    elapsedTime = 0.0;
}

void control2coeffs(double * control_points, double * p)
{
    
    p[3] = control_points[0];
    p[2] = 3*control_points[2]-3*control_points[0];
    p[1] = 3*control_points[4]-6*control_points[2]+3*control_points[0];
    p[0] = control_points[6]-3*control_points[4]+3*control_points[2]-control_points[0];
    p[7] = control_points[1];
    p[6] = 3*control_points[3]-3*control_points[1];
    p[5] = 3*control_points[5]-6*control_points[3]+3*control_points[1];
    p[4] = control_points[7]-3*control_points[5]+3*control_points[3]-control_points[1];

}

void coeffs2control(double * control_points, double * p)
{
    control_points[0] = p[3];
    control_points[1] = p[7];
    control_points[2] = p[2]/3+p[3];
    control_points[3] = p[6]/3+p[7];
    control_points[4] = p[1]/3+2*p[2]/3+p[3];
    control_points[5] = p[5]/3+2*p[6]/3+p[7];
    control_points[6] = p[0]+p[1]+p[2]+p[3];
    control_points[7] = p[4]+p[5]+p[6]+p[7];
}
//Takes Bezier polynomial coeffs in world frame, returns the projected poly coeffs in stereo image frames
void poly_body2image(double * p_body, double * p_left, double * p_right)
{
    double c_body[8];
    double c_left[8];
    double c_right[8];
    coeffs2control(c_body, p_body);

    cp_body2image(c_body, c_left, c_right);

    control2coeffs(c_left,p_left);
    control2coeffs(c_right,p_right);
}
void cp_body2image(double * c_body, double * c_left, double * c_right)
{

    double X, Y, Z;
    for (int i = 0; i <= CURVE_ORDER; i++)
    {
        X = c_body[2*i];
        Y = c_body[2*i+1];
        Z = HEIGHT;

        c_left[2*i] = FX*(Y+0.5*BASELINE)/X+CX;
	c_right[2*i] = FX*(Y-0.5*BASELINE)/X+CX;
	c_left[2*i+1] = FY*(Z/X)+CY;
	c_right[2*i+1] = FY*(Z/X)+CY;
    }
}

void poly_earth2image(double * p_earth, double * p_left, double * p_right, double * euler, double * translation)
{
    double c_earth[8];
    double c_left[8];
    double c_right[8];
    coeffs2control(c_earth, p_earth);

    cp_earth2image(c_earth, c_left, c_right, euler, translation);

    control2coeffs(c_left,p_left);
    control2coeffs(c_right,p_right);
}
void cp_earth2image(double * c_earth, double * c_left, double * c_right, double * euler, double * translation)
{
    for (int i = 0; i <= CURVE_ORDER; i++)
    {
        double xe[] = {c_earth[2*i], c_earth[2*i+1],0};
        double xb[3];
        earth2body(xe, xb, euler, translation);

        c_left[2*i] = FX*(xb[1]+0.5*BASELINE)/xb[0]+CX;
	c_right[2*i] = FX*(xb[1]-0.5*BASELINE)/xb[0]+CX;
	c_left[2*i+1] = FY*(xb[2]/xb[0])+CY;
	c_right[2*i+1] = FY*(xb[2]/xb[0])+CY;
    }
}

void cp_earth2image_single(double * c_earth, double * c_left, double * c_right, double * euler, double * translation)
{
    double xe[] = {c_earth[0], c_earth[1],0};
    double xb[3];
    earth2body(xe, xb, euler, translation);

    c_left[0] = FX*(xb[1]+0.5*BASELINE)/xb[0]+CX;
    c_right[0] = FX*(xb[1]-0.5*BASELINE)/xb[0]+CX;
    c_left[1] = FY*(xb[2]/xb[0])+CY;
    c_right[1] = FY*(xb[2]/xb[0])+CY;
}

void earth2body(double * xe, double * xb, double * euler, double * translation)
{
    xtemp31->data.db[0] = xe[0]-translation[0];
    xtemp31->data.db[1] = xe[1]-translation[1];
    xtemp31->data.db[2] = xe[2]-translation[2];


    double psi = euler[0];
    double theta = euler[1];
    double phi = euler[2];
    generate_Rbe(phi, theta, psi, xtemp33);
    cvMatMul(xtemp33,xtemp31,xtemp31);

    xb[0] = xtemp31->data.db[0];
    xb[1] = xtemp31->data.db[1];
    xb[2] = xtemp31->data.db[2];
}

void body2earth(double * xe, double * xb, double * euler, double * translation)
{
    CvMat * xtemp = cvCreateMat(3,1,CV_64FC1);
    xtemp->data.db[0] = xb[0];
    xtemp->data.db[1] = xb[1];
    xtemp->data.db[2] = xb[2];


    CvMat * R_eb = cvCreateMat(3,3,CV_64FC1);
    double psi = euler[0];
    double theta = euler[1];
    double phi = euler[2];
    generate_Reb(phi, theta, psi, R_eb);
    cvMatMul(R_eb,xtemp,xtemp);

    xtemp->data.db[0] += translation[0];
    xtemp->data.db[1] += translation[1];
    xtemp->data.db[2] += translation[2];

    xe[0] = xtemp->data.db[0];
    xe[1] = xtemp->data.db[1];
    xe[2] = xtemp->data.db[2];
    
    cvReleaseMat(&xtemp);
    cvReleaseMat(&R_eb);
}

void image2body_left(double * body_pt, double * image_pt)
{

    double Z;

        Z = HEIGHT;

        body_pt[0] = FY*Z/(image_pt[1]-CY);
        body_pt[1] = body_pt[0]*(image_pt[0]-CX)/FX-0.5*BASELINE;
}


void image2body_right(double * body_pt, double * image_pt)
{

    double Z;

        Z = HEIGHT;

        body_pt[0] = FY*Z/(image_pt[1]-CY);
        body_pt[1] = body_pt[0]*(image_pt[0]-CX)/FX+0.5*BASELINE;
}

void image2earth_left(double * earth_pt, double * image_pt, double * euler, double * translation)
{

    double Z, body_pt[3];

        Z = -translation[2];

        body_pt[0] = FY*Z/(image_pt[1]-CY);
        body_pt[1] = body_pt[0]*(image_pt[0]-CX)/FX-0.5*BASELINE;
        body_pt[2] = 0.0;

        double earth_pt3[3];
        body2earth(earth_pt3, body_pt, euler, translation);
        earth_pt[0] = earth_pt3[0];
        earth_pt[1] = earth_pt3[1];
}


void image2earth_right(double * earth_pt, double * image_pt, double * euler, double * translation)
{

    double Z, body_pt[3];

        Z = -translation[2];

        body_pt[0] = FY*Z/(image_pt[1]-CY);
        body_pt[1] = body_pt[0]*(image_pt[0]-CX)/FX+0.5*BASELINE;
        body_pt[2] = 0.0;

        double earth_pt3[3];
        
        body2earth(earth_pt3, body_pt, euler, translation);
        earth_pt[0] = earth_pt3[0];
        earth_pt[1] = earth_pt3[1];
}


void closest_bezier_pt(double * p, double * pt, double * nearest_pt, double * out)
{

    //Find coefficients of fifth order polynomial in t, using p

        gettimeofday(&start, NULL);
    double poly[6];

    poly[5] = 3.0*(pow(p[0],2.0)+pow(p[4],2.0));
    poly[4] = 5.0*(p[1]*p[0]+p[5]*p[4]);
    poly[3] = 2*(2*p[2]*p[0]+pow(p[1],2.0)+2*p[6]*p[4]+pow(p[5],2.0));
    poly[2] = 3.0*(p[2]*p[1]+p[0]*(p[3]-pt[0])+p[6]*p[5]+p[4]*(p[7]-pt[1]));
    poly[1] = pow(p[2],2.0)+2*p[1]*(p[3]-pt[0])+pow(p[6],2.0)+2*p[5]*(p[7]-pt[1]);
    poly[0] = p[2]*(p[3]-pt[0])+p[6]*(p[7]-pt[1]);

   //Find the order of the polynomial
    int num_zeros = 0;
    for (int i = 5; i >=0; i--)
    {
        if (poly[i] == 0)
            num_zeros++;
        else
            break;
    }

    int n = 6-num_zeros;


   double roots[2*n-2];
   double poly2[n];

   for (int i = 0; i < n; i++)
       poly2[i] = poly[i];

   try
   {
       int QRresult = gsl_poly_complex_solve (poly2, n, w[n-1], roots);
   }
   catch(...)
   {
       cout << "Root solving failed to converge\n";
   }

       double min_dist_sq = 10000000.0;
       double t_best = 0.0;
   // For all real roots, use the value which yields the closest distance
   for (int i = 0; i < n; i++)
   {
       //If complex component is zero
       if (roots[2*i+1] == 0.0)
       {
           double t = roots[2*i];

           if (t >= 0.0 && t <= 1.0)
           {
                //Determine x and y values
               double xval = p[0]*pow(t,3.0)+p[1]*pow(t,2.0)+p[2]*t+p[3];
               double yval = p[4]*pow(t,3.0)+p[5]*pow(t,2.0)+p[6]*t+p[7];

               double dist_sq = pow(xval-pt[0],2.0)+pow(yval-pt[1],2.0);

               if (dist_sq < min_dist_sq)
               {
                   t_best = t;
                   min_dist_sq = dist_sq;
                   nearest_pt[0] = xval;
                   nearest_pt[1] = yval;
               }
           }
           else
           {
               //Check which endpoint is closer

               double xval0 = p[3];
               double yval0 = p[7];
               double xval1 = p[0]+p[1]+p[2]+p[3];
               double yval1 = p[4]+p[5]+p[6]+p[7];
               double dist_sq0 = pow(xval0-pt[0],2.0)+pow(yval0-pt[1],2.0);
               double dist_sq1 = pow(xval1-pt[0],2.0)+pow(yval1-pt[1],2.0);

               if (dist_sq0 > dist_sq1 && dist_sq1 < min_dist_sq)
               {
                   t_best = 1.0;
                   min_dist_sq = dist_sq1;
                   nearest_pt[0] = xval1;
                   nearest_pt[1] = yval1;
               }
               else if (dist_sq1 > dist_sq0 && dist_sq0 < min_dist_sq)
               {
                   t_best = 0.0;
                   min_dist_sq = dist_sq0;
                   nearest_pt[0] = xval0;
                   nearest_pt[1] = yval0;

               }
           }
       }
   }

           //cout << t_best << "\t";
        gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

        out[0] = pow(min_dist_sq,0.5);
        out[1] = t_best;

}

/*
double closest_bezier_pt(double * p, double t, double * pt, double * nearest_pt)
{
    //Find coefficients of fifth order polynomial in t, using p
        nearest_pt[0] = p[0]*pow(t,3.0)+p[1]*pow(t,2.0)+p[2]*t+p[3];
        nearest_pt[1] = p[4]*pow(t,3.0)+p[5]*pow(t,2.0)+p[6]*t+p[7];

       return pow(pow(nearest_pt[0]-pt[0],2.0)+pow(nearest_pt[1]-pt[1],2.0),0.5);

}
*/

double closest_bezier_pt(double * cp, double t, double * pt, double * nearest_pt)
{
    //Find coefficients of fifth order polynomial in t, using cp
    nearest_pt[0] = 0.0;
    nearest_pt[1] = 0.0;

    for (int i = 0; i <= CURVE_ORDER; i++)
    {
        //nearest_pt[0] += bcoeff[CURVE_ORDER][i]*pow(1-t,CURVE_ORDER-i)*pow(t,i)*cp[2*i];
        //nearest_pt[1] += bcoeff[CURVE_ORDER][i]*pow(1-t,CURVE_ORDER-i)*pow(t,i)*cp[2*i+1];
        nearest_pt[0] += binom[CURVE_ORDER][i]*pow(1-t,CURVE_ORDER-i)*pow(t,i)*cp[2*i];
        nearest_pt[1] += binom[CURVE_ORDER][i]*pow(1-t,CURVE_ORDER-i)*pow(t,i)*cp[2*i+1];
    }

       return pow(pow(nearest_pt[0]-pt[0],2.0)+pow(nearest_pt[1]-pt[1],2.0),0.5);

}


int binomial(int n, int k)
{
    if (k == 0)
        return 1;
    else if (n == 0)
        return 0;
    else
        return (binomial(n-1,k)+binomial(n-1,k-1));
}

void CurveFittingClass::generateMeasurements(CvMat * state_actual, double t_start, CvMat * featuresL, CvMat * featuresR, int current_curves)
{
    double c_earth[8];
    double c_left[8];
    double c_right[8];
    double euler[] = {state_actual->data.db[5], state_actual->data.db[4], state_actual->data.db[3]};
    double translation[] = {state_actual->data.db[0], state_actual->data.db[1], state_actual->data.db[2]};

    int num_features = featuresL->rows/4;


    // For each of the 4 curves
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            c_earth[2*j] = state_actual->data.db[j+i*8+ROBOT_STATE_SIZE+16*current_curves];
            c_earth[2*j+1] = state_actual->data.db[j+i*8+ROBOT_STATE_SIZE+4+16*current_curves];
        }

        cp_earth2image(&(c_earth[0]), &(c_left[0]), &(c_right[0]), &(euler[0]), &(translation[0]));
        //cout << "CPs:\n";
        for (int f = 0; f < 8; f++)
        {
            //printf("%.2f\t%.2f\t%.2f\n",c_earth[f],c_left[f],c_right[f]);
        }


        double t = 0.0;
        int k_start = (num_features-1)*t_start;
        if (i<2)
        {
            for (int k = k_start; k < (num_features-1); k++)
            {
                t = k/((double)(num_features-1));
                //Generate curve points in image frames
                for (int j = 0; j <= 3; j++)
                {
                    featuresL->data.db[2*(k-k_start)+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_left[2*j];
                    featuresL->data.db[2*(k-k_start)+1+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_left[2*j+1];
                    featuresR->data.db[2*(k-k_start)+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_right[2*j];
                    featuresR->data.db[2*(k-k_start)+1+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_right[2*j+1];
                }
            }
        }
        else
        {
            for (int k = 0; k <= k_start; k++)
            {
                t = k/((double)(num_features-1));
                //Generate curve points in image frames
                for (int j = 0; j <= 3; j++)
                {
                    featuresL->data.db[2*(k+(num_features-1)-k_start)+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_left[2*j];
                    featuresL->data.db[2*(k+(num_features-1)-k_start)+1+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_left[2*j+1];
                    featuresR->data.db[2*(k+(num_features-1)-k_start)+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_right[2*j];
                    featuresR->data.db[2*(k+(num_features-1)-k_start)+1+(i%2)*2*num_features] += binom[3][j]*pow(1-t,3-j)*pow(t,j)*c_right[2*j+1];
                }

            }
        }
        
    }

    for (int i = 0; i < featuresL->rows/2; i++)
    {
        double rand1 = (double)(rand())/(double)(RAND_MAX);
        double rand2 = (double)(rand())/(double)(RAND_MAX);
        double rands1 = PIX_STDEV*pow(-2.0*log(rand1),0.5)*cos(2.0*PI*rand2);
        double rands2 = PIX_STDEV*pow(-2.0*log(rand1),0.5)*sin(2.0*PI*rand2);

        featuresL->data.db[2*i] += rands1;
        featuresL->data.db[2*i+1] += rands2;
        rand1 = (double)(rand())/(double)(RAND_MAX);
        rand2 = (double)(rand())/(double)(RAND_MAX);
        rands1 = PIX_STDEV*pow(-2.0*log(rand1),0.5)*cos(2.0*PI*rand2);
        rands2 = PIX_STDEV*pow(-2.0*log(rand1),0.5)*sin(2.0*PI*rand2);
        featuresR->data.db[2*i] += rands1;
        featuresR->data.db[2*i+1] += rands2;
    }
}

void printMatrix(CvMat * matrix)
{
    int cols = matrix->cols;
    int rows = matrix->rows;
    cout << endl;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            printf("%.3f\t",matrix->data.db[cols*i+j]);
        }
        cout << endl;
    }
    cout << endl;

    
}

void LinearLeastSquares(CvMat * A, CvMat * b, CvMat * x)
{
    CvMat * At = cvCreateMat(A->cols,A->rows,CV_64FC1);
    CvMat * temp = cvCreateMat(A->cols,A->cols,CV_64FC1);

    cvTranspose(A,At);
    cvMatMul(At,A,temp);
    cvInvert(temp,temp,CV_SVD);

    cvMatMul(temp,At,At);
    cvMatMul(At,b,x);

    cvReleaseMat(&At);
    cvReleaseMat(&temp);
}
