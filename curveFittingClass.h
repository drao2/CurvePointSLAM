/* 
 * File:   levmar.h
 * Author: dushyant
 *
 * Created on June 30, 2011, 2:22 PM
 */
#include "common.h"

#include "levmar-2.5/levmar.h"
#include "levmar-2.5/misc.h"

#ifndef CURVEFITTINGCLASS_H
#define	CURVEFITTINGCLASS_H

#define HEIGHT  1.0
#define CURVE_ORDER 3

#define PIX_STDEV   0.75     //For error in measurements

#include "displayClass.h"

struct mydata{
    double *orig_meas;
    double *t;
    int *curve_num;
    int num_left1;
    int num_right1;
    int num_left2;
    int num_right2;
    DisplayClass * disp;
};

int binomial(int n, int k);


void printMatrix(CvMat * matrix);


/* model to be fitted to measurements: 'second set of features' = f( 'first set' ) */
void modelfunc(double *cp, double *x, int m, int n, void *datax);
/* Jacobian of expfunc() */
void jacmodelfunc(double *p, double *jac, int m, int n, void *datax);

void control2coeffs(double * control_points, double * p);
void coeffs2control(double * control_points, double * p);
//Takes Bezier polynomial coeffs in world frame, returns the projected poly coeffs in stereo image frames
void poly_body2image(double * p_body, double * p_left, double * p_right);
void cp_body2image(double * c_body, double * c_left, double * c_right);

void poly_earth2image(double * p_earth, double * p_left, double * p_right, double * euler, double * translation);
void cp_earth2image(double * c_earth, double * c_left, double * c_right, double * euler, double * translation);

void cp_earth2image_single(double * c_earth, double * c_left, double * c_right, double * euler, double * translation);

void earth2body(double * xe, double * xb, double * euler, double * translation);

void body2earth(double * xe, double * xb, double * euler, double * translation);

void image2body_left(double * body_pt, double * image_pt);


void image2body_right(double * body_pt, double * image_pt);

void image2earth_left(double * earth_pt, double * image_pt, double * euler, double * translation);
void image2earth_right(double * earth_pt, double * image_pt, double * euler, double * translation);


void closest_bezier_pt(double * p, double * pt, double * nearest_pt, double * out);
double closest_bezier_pt(double * cp, double t, double * pt, double * nearest_pt);

int binomial(int n, int k);


void LinearLeastSquares(CvMat * A, CvMat * b, CvMat * x);

class CurveFittingClass
{
public:
	CurveFittingClass();
	~CurveFittingClass();

    void fit_curve(double * p, CvMat ** features, CvMat ** features2, DisplayClass * display);

    void generateMeasurements(CvMat * state_actual, double t, CvMat * featuresL, CvMat * featuresR, int current_curves);


        void cleanup_and_group_edges(std::vector<CvPoint> ** featuresL, std::vector<CvPoint> ** featuresR, CvMat * featuresLnew, CvMat * featuresRnew);      //Function to cleanup edge points and make sure the same amount of each curve is observed in both images

        float getTime();
	void resetTime();



        double fitting_error;


        CvMat * start_pt1;
        CvMat * start_pt2;
        CvMat * mid_pt1;
        CvMat * mid_pt2;
        CvMat * end_pt1;
        CvMat * end_pt2;
        CvMat * centroid;
        CvMat * Rot1;
        CvMat * Rot2;
        CvMat * Rot3;


        CvMat * M;
        CvMat * W;
        CvMat * U;
        CvMat * V;
        CvMat *normal;
        CvMat *origin;
        CvMat *Rot;




};
#endif	/* POSEESTIMATION_H */


