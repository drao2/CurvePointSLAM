#ifndef _COMMON_H
#define	_COMMON_H

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <cxcore.h>
#include <cxmisc.h>
#include <ml.h>
//#include <cvwimage.h>
//#include <video/tracking.hpp>
//#include <video/video.hpp>

//#include <opencv.hpp>


#include <deque>

//#define NO_SLAM

#define CRYSTAL_LAKE

#define MEAS_STDEV 0.1
#define ONLY_ADD_STATE 0x01
#define ONLY_UPDATE_ONE_STATE 0x02
#define UPDATE_TWO_STATES 0x04
#define ADD_AND_UPDATE_STATES 0x08
#define DISCARD_MEASUREMENT 0x10
#define ADD_FIRST_STATES 0x20
#define ONLY_UPDATE_ONE_STATE_PREV 0x40

#define MAP_IMAGE_WIDTH     640.0
#define MAP_IMAGE_HEIGHT    480.0
#define NEIGHBORHOOD_IMAGE_WIDTH     330
#define NEIGHBORHOOD_IMAGE_HEIGHT    110
#define MAP_PHYSICAL_WIDTH     200.0
#define MAP_PHYSICAL_HEIGHT    150.0

#define PIC_WIDTH	320
#define PIC_HEIGHT	240

#define R11     0.9998790
#define R12     0.0152133
#define R13     -0.0032504
#define R21     -0.0151523
#define R22     0.9997230
#define R23     0.0180092
#define R31     0.0035235
#define R32     -0.0179578
#define R33     0.9998325

#define T1     -535.7736
#define T2     11.4829
#define T3     9.1717

/*#define FX0     525.44
#define FY0     522.76
#define CX0     309.34
#define CY0     227.12

#define FX1     528.85
#define FY1     528.82
#define CX1     311.45
#define CY1     226.08*/





#define FX0     521.22/2.0
#define FY0     518.32/2.0
#define CX0     296.08/2.0-1.0
#define CY0     236.17/2.0

#define FX1     527.03/2.0
#define FY1     524.73/2.0
#define CX1     320.95/2.0+1.0
#define CY1     228.81/2.0

#define POINT_OFFSET    0.0



#define FX ((FX0+FX1)/2.0)
#define FY ((FY0+FY1)/2.0)
#define CX ((CX0+CX1)/2.0)
#define CY ((CY0+CY1)/2.0)

#define D01     0//0.04587/2
#define D02     0//-0.13701/2
#define D03     0//-0.00223/2
#define D04     0//-0.00032/2
#define D05     0.0

#define D11     0.04479/2
#define D12     -0.13120/2
#define D13     -0.00169/2
#define D14     -0.00067/2
#define D15     0.0


#define BASELINE    0.535

#define NUM_CAMERAS	2
#define LEFT	0
#define RIGHT	1


#define PI  3.1415926535

#define SIGN(X) (((X) >= 0.0) ? 1.0 : -1.0)
//#define MIN(X,Y)    ((Y <= X) ? (Y) : (X))
//#define MAX(X,Y)    ((Y >= X) ? (Y) : (X))
#define DIST(X,Y)       ( pow ( pow( X.x - Y.x , 2.0 ) + pow( X.y - Y.y , 2.0 ) , 0.5 ) )

#define BOX(X,Y,Z)      ( (X) < (Y)? (Y) : ( (X)>(Z)? Z : X ) )



#define NUM_TRACK_PTS   5


//#define USE_SIMULATION
#define USE_VIDEO
//#define SAVE_VIDEO

#define VIDEO_FILENAME "cam24%d.avi"


//Macros
#define CVPOINT_DIST(X,Y)       ( pow( ( pow( (X.x-Y.x),2.0 ) + pow( (X.y-Y.y),2.0 ) ), 0.5 ) )

using namespace std;


struct poseStruct{
    std::deque <CvMat *> rotation;
    std::deque <CvMat *> translation;
};

extern unsigned int binom[50][50];


struct curveCharacteristics{
    int last_curve;
    int next_curve;
    CvPoint2D32f start_pt;
    CvPoint2D32f end_pt;
    bool start_in_view;
    bool end_in_view;
};



inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels)
{
    if ( *img != NULL ) return;
    *img = cvCreateImage( size, depth, channels );
    if ( *img == NULL )
    {
    fprintf(stderr, "Error: Couldn't allocate image. Out of memory?\n");
    exit(-1);
    }
}




inline static void generate_Rbe(double phi, double theta, double psi, CvMat * R_be)
{
    R_be->data.db[0] = cos(psi)*cos(theta);
    R_be->data.db[3] = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
    R_be->data.db[6] = cos(psi)*sin(theta)*cos(phi)-sin(psi)*sin(phi);
    R_be->data.db[1] = sin(psi)*cos(theta);
    R_be->data.db[4] = sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi);
    R_be->data.db[7] = sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    R_be->data.db[2] = -sin(theta);
    R_be->data.db[5] = cos(theta)*sin(phi);
    R_be->data.db[8] = cos(theta)*cos(phi);

}


inline static void generate_Reb(double phi, double theta, double psi, CvMat * R_eb)
{
    R_eb->data.db[0] = cos(psi)*cos(theta);
    R_eb->data.db[1] = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
    R_eb->data.db[2] = cos(psi)*sin(theta)*cos(phi)-sin(psi)*sin(phi);
    R_eb->data.db[3] = sin(psi)*cos(theta);
    R_eb->data.db[4] = sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi);
    R_eb->data.db[5] = sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    R_eb->data.db[6] = -sin(theta);
    R_eb->data.db[7] = cos(theta)*sin(phi);
    R_eb->data.db[8] = cos(theta)*cos(phi);
    
}

inline static void printMatrix(CvMat * matrix)
{
    int cols = matrix->cols;
    int rows = matrix->rows;
    cout << endl;
    if (cols == 1)
    {
        cols = rows;
        rows = 1;
    }
    
    for (int i = 0; i < rows; i++)
    {
        printf("%d:\t",i);
        for (int j = 0; j < cols; j++)
        {
            if(matrix->data.db[cols*i+j] >= 0.0)
                printf(" %.2f ",matrix->data.db[cols*i+j]);
            else
                printf("%.2f ",matrix->data.db[cols*i+j]);
        }
        cout << endl;
    }
    cout << endl;
}




#endif	/* _COMMON_H */

