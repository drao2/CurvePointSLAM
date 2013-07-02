#include "kalmanClass.h"
#include "displayClass.h"

using namespace std;

KalmanFilter::KalmanFilter()
{
        //Matrices related to curve splitting
        Ainv = newMatrix(4,4,CV_64FC1);
        Ainvzx = newMatrix(4,1,CV_64FC1);
        Ainvzy = newMatrix(4,1,CV_64FC1);
        zx = newMatrix(4,1,CV_64FC1);
        zy = newMatrix(4,1,CV_64FC1);


	A1 = newMatrix(4,4,CV_64FC1);
	A2 = newMatrix(4,4,CV_64FC1);
	B1 = newMatrix(4,4,CV_64FC1);
	B2 = newMatrix(4,4,CV_64FC1);
        
        //Measurement model matrices
	H = newMatrix(8,8,CV_64FC1);
	H1 = newMatrix(8,8+ROBOT_STATE_SIZE,CV_64FC1);
	H1t = newMatrix(8+ROBOT_STATE_SIZE,8,CV_64FC1);
	Hinv = newMatrix(8,8,CV_64FC1);
	K1 = newMatrix(8+ROBOT_STATE_SIZE,8,CV_64FC1);
	delP1 = newMatrix(8+ROBOT_STATE_SIZE,8+ROBOT_STATE_SIZE,CV_64FC1);
	R1 = newMatrix(8,8,CV_64FC1);
	R_pts = newMatrix(3,3,CV_64FC1);
	S1 = newMatrix(8,8,CV_64FC1);
	z1 = newMatrix(8,1,CV_64FC1);
	z2 = newMatrix(16,1,CV_64FC1);
	z_hat1 = newMatrix(8,1,CV_64FC1);
	delx1 = newMatrix(8+ROBOT_STATE_SIZE,1,CV_64FC1);
	H2 = newMatrix(16,16+ROBOT_STATE_SIZE,CV_64FC1);
	H2t = newMatrix(16+ROBOT_STATE_SIZE,16,CV_64FC1);
	K2 = newMatrix(16+ROBOT_STATE_SIZE,16,CV_64FC1);
	delP2 = newMatrix(16+ROBOT_STATE_SIZE,16+ROBOT_STATE_SIZE,CV_64FC1);
	delx2 = newMatrix(16+ROBOT_STATE_SIZE,1,CV_64FC1);
        Rot = newMatrix(8,8,CV_64FC1);
        Rotderiv = newMatrix(8,8,CV_64FC1);
        
        
        //Process model matrices
        Reb = newMatrix(3,3,CV_64FC1);
        Rebt = newMatrix(3,3,CV_64FC1);
        Reb_phi = newMatrix(3,3,CV_64FC1);
        Reb_theta = newMatrix(3,3,CV_64FC1);
        Reb_psi = newMatrix(3,3,CV_64FC1);
        
        //Temporary matrices of various sizes
	temp8 = newMatrix(8,8,CV_64FC1);
	temp81 = newMatrix(8,1,CV_64FC1);
	temp4 = newMatrix(4,4,CV_64FC1);
	temp41 = newMatrix(4,1,CV_64FC1);
	temp43 = newMatrix(4,3,CV_64FC1);
	temp31 = newMatrix(3,1,CV_64FC1);
	temp312 = newMatrix(3,1,CV_64FC1);
	temp33 = newMatrix(3,3,CV_64FC1);
        
	Vcov = newMatrix(3,1,CV_64FC1);
	Wcov = newMatrix(3,1,CV_64FC1);
            
        Vcov->data.db[0] = VX_COV;
        Vcov->data.db[1] = VY_COV;
        Vcov->data.db[2] = VZ_COV;
        Wcov->data.db[0] = WX_COV;
        Wcov->data.db[1] = WY_COV;
        Wcov->data.db[2] = WZ_COV;

	Pcurrent8 = newMatrix(8,8,CV_64FC1);
	xcurrent8 = newMatrix(8,1,CV_64FC1);
	P = newMatrix(16+ROBOT_STATE_SIZE,16+ROBOT_STATE_SIZE,CV_64FC1);
	x = newMatrix(16+ROBOT_STATE_SIZE,1,CV_64FC1);
	xOOP = newMatrix(3,1,CV_64FC1);
	POOP = newMatrix(3,3,CV_64FC1);
	QOOP = newMatrix(3,3,CV_64FC1);
	ROOP = newMatrix(3,3,CV_64FC1);
        cvSetZero(x);

        ROOP->data.db[0] = PHI_MEAS_COV;
        ROOP->data.db[4] = THETA_MEAS_COV;
        ROOP->data.db[8] = Z_MEAS_COV;

	Fpose = newMatrix(ROBOT_STATE_SIZE,ROBOT_STATE_SIZE,CV_64FC1);
	Qpose = newMatrix(ROBOT_STATE_SIZE,ROBOT_STATE_SIZE,CV_64FC1);


	Gx = newMatrix(8,ROBOT_STATE_SIZE,CV_64FC1);
	Gz = newMatrix(8,8,CV_64FC1);
        
        Prr = newMatrix(ROBOT_STATE_SIZE,ROBOT_STATE_SIZE,CV_64FC1);

        // R is diagonal matrix with measurement covariance
        for (int i = 0; i < 8; i++)
        {
            R1->data.db[9*i] = MEAS_COV;
        }
        for (int i = 0; i < 3; i++)
        {
            R_pts->data.db[4*i] = PT_MEAS_COV;
        }

        cvSetZero(Fpose);
    
    for (int i = 0; i< ROBOT_STATE_SIZE; i++)
    {
        Fpose->data.db[(ROBOT_STATE_SIZE+1)*i] = 1.0;
    }
    
    for (int i = 0; i< 6; i++)
    {
        Fpose->data.db[ROBOT_STATE_SIZE*i+(i+6)] = DT;
    }
        
        num_curves = 0;
        num_points = 0;
        cvSetZero(x);   
        
    x->data.db[6] = VREAL;
}
KalmanFilter::~KalmanFilter()
{
    //Release all matrices
    cvReleaseMat(&A1);
    cvReleaseMat(&A2);
    cvReleaseMat(&B1);
    cvReleaseMat(&B2);
    cvReleaseMat(&H1);
    cvReleaseMat(&H1t);
    cvReleaseMat(&Hinv);
    cvReleaseMat(&K1);
    cvReleaseMat(&delP1);
    cvReleaseMat(&R1);
    cvReleaseMat(&S1);
    cvReleaseMat(&z1);
    cvReleaseMat(&z_hat1);
    cvReleaseMat(&delx1);
    cvReleaseMat(&H2);
    cvReleaseMat(&H2t);
    cvReleaseMat(&K2);
    cvReleaseMat(&delP2);
    cvReleaseMat(&z2);
    cvReleaseMat(&delx2);
    cvReleaseMat(&temp8);
    cvReleaseMat(&temp4);
    cvReleaseMat(&temp41);
    cvReleaseMat(&temp43);
    cvReleaseMat(&temp31);
    cvReleaseMat(&x);
    cvReleaseMat(&P);
    cvReleaseMat(&xcurrent8);
    cvReleaseMat(&Pcurrent8);

    cvReleaseMat(&Fpose);
    cvReleaseMat(&Qpose);

}

//Process model of the EKF, updates state and covariance
void KalmanFilter::PredictKF()
{
    //Initialise matrices containing parts of the cov mat
    CvMat * Pri = cvCreateMat(ROBOT_STATE_SIZE,num_curves*8+num_points*3,CV_64FC1);
    cvSetZero(Prr);
    cvSetZero(Pri);
    
    //Initialise rotation mat    
    double phi = x->data.db[3];
    double theta = x->data.db[4];
    double psi = x->data.db[5];
    
    Reb->data.db[0] = cos(psi)*cos(theta);
    Reb->data.db[1] = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
    Reb->data.db[2] = cos(psi)*sin(theta)*cos(phi)-sin(psi)*sin(phi);
    Reb->data.db[3] = sin(psi)*cos(theta);
    Reb->data.db[4] = sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi);
    Reb->data.db[5] = sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    Reb->data.db[6] = -sin(theta);
    Reb->data.db[7] = cos(theta)*sin(phi);
    Reb->data.db[8] = cos(theta)*cos(phi);
    
    cvTranspose(Reb,Rebt);
    

    //Process covariance
    cvSetZero(Qpose);
    cvSetZero(temp33);
    temp33->data.db[0] = pow(VX_COV,2.0);
    temp33->data.db[4] = pow(VY_COV,2.0);
    temp33->data.db[8] = pow(VZ_COV,2.0);
    
    cvMatMul(Reb,temp33,temp33);
    cvMatMul(temp33,Rebt,temp33);
    
    //Cov(r,r) = ...
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cvmSet(Qpose,i,j, pow(DT,3.0)*cvmGet(temp33,i,j));
            cvmSet(Qpose,i,j+6, pow(DT,2.0)*cvmGet(temp33,i,j));
            cvmSet(Qpose,j+6,i, pow(DT,2.0)*cvmGet(temp33,i,j));
            cvmSet(Qpose,i+6,j+6, pow(DT,1.0)*cvmGet(temp33,i,j));
        }
    }
    
    cvSetZero(temp33);
    cvmSet(temp33,0,0, pow(WX_COV,2.0));
    cvmSet(temp33,1,1, pow(WY_COV,2.0));
    cvmSet(temp33,2,2, pow(WZ_COV,2.0));
    
    //Cov(Psi,omega) = ...
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cvmSet(Qpose,i+3,j+3, pow(DT,3.0)*cvmGet(temp33,i,j));
            cvmSet(Qpose,i+3,j+9, pow(DT,2.0)*cvmGet(temp33,i,j));
            cvmSet(Qpose,j+9,i+3, pow(DT,2.0)*cvmGet(temp33,i,j));
            cvmSet(Qpose,i+9,j+9, pow(DT,1.0)*cvmGet(temp33,i,j));
        }
    }
    
    //Get current covariance parts
    for (int i = 0; i < ROBOT_STATE_SIZE; i++)
    {
        for (int j = 0; j < ROBOT_STATE_SIZE; j++)
        {
            cvmSet(Prr,i,j,cvmGet(P,i,j));
        }
        for (int j = ROBOT_STATE_SIZE; j < num_curves*8+num_points*3+ROBOT_STATE_SIZE; j++)
        {
            cvmSet(Pri,i,j-ROBOT_STATE_SIZE,cvmGet(P,i,j));
        }

    }

    //Update covariance parts
    cvMatMul(Fpose,Pri,Pri);
    cvMatMul(Fpose,Prr,Prr);
    cvTranspose(Fpose,Fpose);
    cvMatMul(Prr,Fpose,Prr);
    cvAdd(Prr,Qpose,Prr);
    cvTranspose(Fpose,Fpose);


    //Set covariance parts
    for (int i = 0; i < ROBOT_STATE_SIZE; i++)
    {
        for (int j = 0; j < ROBOT_STATE_SIZE; j++)
        {
            cvmSet(P,i,j,cvmGet(Prr,i,j));
        }
        for (int j = ROBOT_STATE_SIZE; j < num_curves*8+num_points*3+ROBOT_STATE_SIZE; j++)
        {
            cvmSet(P,i,j,cvmGet(Pri,i,j-ROBOT_STATE_SIZE));
            cvmSet(P,j,i,cvmGet(Pri,i,j-ROBOT_STATE_SIZE));
        }

    }
    
    //Predict state    
    x->data.db[0] += x->data.db[6]*DT;
    x->data.db[1] += x->data.db[7]*DT;
    x->data.db[2] += x->data.db[8]*DT;
    
    x->data.db[3] += x->data.db[9]*DT;
    x->data.db[4] += x->data.db[10]*DT;
    x->data.db[5] += x->data.db[11]*DT;
    cvReleaseMat(&Pri);
}

//Adds the first curve states and initialises covariance
void KalmanFilter::AddFirstStates(CvMat * measurement)
{
    
    cvSetZero(P);
    cvSetZero(x);
    
    //First Curve control points and their initial covariance
    for (int i = 0; i < 16; i++)
    {
        cvmSet(P, i+ROBOT_STATE_SIZE, i+ROBOT_STATE_SIZE, pow(MEAS_COV,2.0));
        x->data.db[i+ROBOT_STATE_SIZE] = measurement->data.db[i];
    }
    
    //z, phi and theta
    x->data.db[2] = measurement->data.db[16];    
    x->data.db[3] = measurement->data.db[17];    
    x->data.db[4] = measurement->data.db[18];
    x->data.db[6] = VREAL;
    
    cvmSet(P, 2, 2, 0.1);
    cvmSet(P, 3, 3, 0.1);
    cvmSet(P, 4, 4, 0.1);
    cvmSet(P, 0, 0, 0.0000001);
    cvmSet(P, 1, 1, 0.0000001);
    cvmSet(P, 5, 5, 0.0000001);
    cvmSet(P, 6, 6, 0.1);
    cvmSet(P, 7, 7, 0.1);
    cvmSet(P, 8, 8, 0.1);
    cvmSet(P, 9, 9, 0.1);
    cvmSet(P, 10, 10, 0.1);
    cvmSet(P, 11, 11, 0.1);

    num_curves = 2;
    curve_inds.push_back(ROBOT_STATE_SIZE);     //Indices specifying where in the state vector each curve is
    curve_inds.push_back(ROBOT_STATE_SIZE+8);

}

//Adds a new curve state (8dim) and initialises its covariance
void KalmanFilter::AddNewCurve(CvMat * z, CvMat * A)
{
	gettimeofday(&start, NULL);
        
        double Tx = x->data.db[0];
        double Ty = x->data.db[1];
        double psi = x->data.db[5];
        cvSetZero(H);
        cvSetZero(temp8);
        cvSetZero(temp81);
        cvSetZero(Rot);
        cvSetZero(Rotderiv);
        
        CvMat * Gznum = newMatrix(8,8,CV_64FC1);
        CvMat * Gxnum = newMatrix(8,ROBOT_STATE_SIZE,CV_64FC1);
        getGxCurveNumeric(Gxnum,z,x,A);
        getGzCurveNumeric(Gznum,z,x,A);
        
        //Determine rot matrix for control points
        for (int i = 0; i < 4; i++)
        {
            cvmSet(Rot,i,i,cos(psi));
            cvmSet(Rot,i+4,i+4,cos(psi));
            cvmSet(Rot,i+4,i,sin(psi));
            cvmSet(Rot,i,i+4,-sin(psi));
            cvmSet(Rotderiv,i,i,-sin(psi));
            cvmSet(Rotderiv,i+4,i+4,-sin(psi));
            cvmSet(Rotderiv,i+4,i,cos(psi));
            cvmSet(Rotderiv,i,i+4,-cos(psi));
        }

        num_curves++;
        
        //Data structures
        int existing_size = P->rows;
        CvMat * PN1N1 = newMatrix(8,8,CV_64FC1);
        CvMat * PN1r = newMatrix(8,ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * PN1i = newMatrix(8,existing_size-ROBOT_STATE_SIZE,CV_64FC1); //For curve terms only
        CvMat * Pri = newMatrix(ROBOT_STATE_SIZE,existing_size-ROBOT_STATE_SIZE,CV_64FC1); //For curve AND terms only
        CvMat * ones = newMatrix(4,1,CV_64FC1);
        CvMat * temp58 = newMatrix(ROBOT_STATE_SIZE,8,CV_64FC1);
        CvMat * temp5 = newMatrix(ROBOT_STATE_SIZE,ROBOT_STATE_SIZE,CV_64FC1);
        ones->data.db[0] = 1.0;
        ones->data.db[1] = 1.0;
        ones->data.db[2] = 1.0;
        ones->data.db[3] = 1.0;

        //Resize P and x
        CvMat * Pcopy = cvCloneMat(P);
        CvMat * xcopy = cvCloneMat(x);
        cvReleaseMat(&P);
        cvReleaseMat(&x);
        P = newMatrix(existing_size+8,existing_size+8,CV_64FC1);
        x = newMatrix(existing_size+8,1,CV_64FC1);
        
        //Find Prr and Pri first
        for (int i = 0; i < ROBOT_STATE_SIZE; i++)
        {
            for (int j = 0; j < ROBOT_STATE_SIZE; j++)
                cvmSet(Prr,i,j,cvmGet(Pcopy,i,j));
            for (int j = 0; j < existing_size-ROBOT_STATE_SIZE; j++)
                cvmSet(Pri,i,j,cvmGet(Pcopy,i,j+ROBOT_STATE_SIZE));
        }
        
        //Copy the robot state and existing landmarks 
        for (int i = 0; i < existing_size; i++)
        {
            for (int j = 0; j < existing_size; j++)
            {
                cvmSet(P,i,j,cvmGet(Pcopy,i,j));
            }
            cvmSet(x,i,0, cvmGet(xcopy,i,0));
        }
        
        cvReleaseMat(&xcopy);
        cvReleaseMat(&Pcopy);

        cvSetZero(Pcurrent8);
        cvSetZero(xcurrent8);
        cvSetZero(H);
        
        
        //Calcs to determine H and Hinv matrices
        for (int i = 0; i < 4; i++)
        {
            cvmSet( zx, i, 0, z->data.db[i]);
            cvmSet( zy, i, 0, z->data.db[i+4]);

        }

        cvInvert(A,Ainv,CV_SVD);
        cvMatMul(Ainv,zx,Ainvzx);
        cvMatMul(Ainv,zy,Ainvzy);
        
        //Determine new H and add new curve
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                cvmSet( H, i, j, A->data.db[4*i+j]);
                cvmSet( H, i+4, j+4, A->data.db[4*i+j]);
                cvmSet( Hinv, i, j, Ainv->data.db[4*i+j]);
                cvmSet( Hinv, i+4, j+4, Ainv->data.db[4*i+j]);
            }

        }
        cvMatMul(Rot,z,temp81);
        for (int i = 0; i < 4; i++)
            temp81->data.db[i] += Tx;
        for (int i = 4; i < 8; i++)
            temp81->data.db[i] += Ty;
        cvMatMul(Hinv,temp81,xcurrent8);        //This is the new curve to add
        
        cvMatMul(Ainv,ones,temp41);

        //Figure out covariances Gz and Gx to add new covariance blocks
        cvMatMul(Hinv,Rot,Gz);

        cvMatMul(Rotderiv,z,temp81);
        cvMatMul(Hinv,temp81,temp81);
        cvSetZero(Gx);

        for (int i = 0; i < 4; i++)
        {
            cvmSet(Gx,i,0, temp41->data.db[i]);
            cvmSet(Gx,i+4,0, 0.0);
            cvmSet(Gx,i,1, 0.0);
            cvmSet(Gx,i+4,1, temp41->data.db[i]);
            cvmSet(Gx,i,5, temp81->data.db[i]);
            cvmSet(Gx,i+4,5, temp81->data.db[i+4]);
        }

        //Get blocks PN1N1, PN1r and PN1i
        cvTranspose(Gx,temp58);
        cvMatMul(Prr,temp58,temp58);
        cvMatMul(Gx,temp58,Pcurrent8);

        cvTranspose(Gz,temp8);
        cvMatMul(R1,temp8,temp8);
        cvMatMul(Gz,temp8,temp8);
        cvAdd(Pcurrent8,temp8,PN1N1);

        cvMatMul(Gx,Pri,PN1i);
        
        cvTranspose(Prr,temp5);
        cvMatMul(Gx,temp5,PN1r);

        //Expand array and add new bits
        curve_inds.push_back(existing_size);    //Store index of new curve
        
        for (int i = 0; i < 8; i++)
        {
            cvmSet(x,existing_size+i,0, cvmGet(xcurrent8,i,0));
            
            for (int j = 0; j < 8; j++)
            {
                cvmSet( P, i+existing_size, j+existing_size, cvmGet(PN1N1,i,j) );
            }
            
            for (int j = 0; j < existing_size-ROBOT_STATE_SIZE; j++)
            {
                cvmSet( P, i+existing_size, j+ROBOT_STATE_SIZE, cvmGet(PN1i,i,j) );
                cvmSet( P, j+ROBOT_STATE_SIZE, i+existing_size, cvmGet(PN1i,i,j) );
            }

            for(int j = 0; j < ROBOT_STATE_SIZE; j++)
            {
                cvmSet( P, i+existing_size, j, cvmGet(PN1r,i,j) );
                cvmSet( P, j, i+existing_size, cvmGet(PN1r,i,j) );
            }
        }

        cvReleaseMat(&Pri);
        cvReleaseMat(&PN1i);
        cvReleaseMat(&PN1N1);
        cvReleaseMat(&PN1r);
        cvReleaseMat(&ones);
        cvReleaseMat(&temp58);
        cvReleaseMat(&temp5);
        
        //printMatrix(P);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

//Adds a bunch of new point states (3dim each) and initialise their covariance
void KalmanFilter::AddNewPoints(double * measurements, int n_pts)
{
    gettimeofday(&start, NULL);
    for (int i = 0; i < n_pts*3; i++)
        cout << " " << measurements[i];
    cout << endl;
    
    //Only proceed if we have points to add
    if (n_pts)
    {        
        double phi = x->data.db[3];
        double theta = x->data.db[4];
        double psi = x->data.db[5];

        //Resize P and x, and copy existing entries over
        int existing_size = P->rows;
        CvMat * Pcopy = cvCloneMat(P);
        CvMat * xcopy = cvCloneMat(x);
        cvReleaseMat(&P);
        cvReleaseMat(&x);
        P = newMatrix(existing_size+n_pts*3,existing_size+n_pts*3,CV_64FC1);
        x = newMatrix(existing_size+n_pts*3,1,CV_64FC1);

        for (int i = 0; i < existing_size; i++)
        {
            for (int j = 0; j < existing_size; j++)
            {
                cvmSet(P,i,j,cvmGet(Pcopy,i,j));
            }
            x->data.db[i] = xcopy->data.db[i];
        }

        cvReleaseMat(&xcopy);
        cvReleaseMat(&Pcopy);

        //Set new landmark states, find Gx and Gz, and fill in covariance blocks, all in one
        //Gx needs to be computed separately for each landmark, since the angular terms are coupled with xb!
        CvMat * xb = newMatrix(3,1,CV_64FC1);
        CvMat * temp31 = newMatrix(3,1,CV_64FC1);
        CvMat * temp33 = newMatrix(3,3,CV_64FC1);
        CvMat * R_eb = newMatrix(3,3,CV_64FC1);
        CvMat * R_eb_derivs[3];
        R_eb_derivs[0] = newMatrix(3,3,CV_64FC1);
        R_eb_derivs[1] = newMatrix(3,3,CV_64FC1);
        R_eb_derivs[2] = newMatrix(3,3,CV_64FC1);
        CvMat * angle_deriv = newMatrix(3,1,CV_64FC1);
        generate_Reb(phi, theta, psi, R_eb);
        get_Reb_derivs(phi, theta, psi, R_eb_derivs[0], R_eb_derivs[1], R_eb_derivs[2]);
        
        CvMat * Gx = newMatrix(3,ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * Gxnum = newMatrix(3,ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * GxT = newMatrix(ROBOT_STATE_SIZE,3,CV_64FC1);
        CvMat * Gz = newMatrix(3,3,CV_64FC1);
        CvMat * Gznum = newMatrix(3,3,CV_64FC1);
        CvMat * GzT = newMatrix(3,3,CV_64FC1);
        
        //Find Prr and Pri, to use later in calculating the new covariances
        CvMat * PN1N1 = newMatrix(3,3,CV_64FC1);
        CvMat * PN1r = newMatrix(3,ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * PN1i = newMatrix(3,existing_size-ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * Pri = newMatrix(ROBOT_STATE_SIZE,existing_size-ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * PrrT = newMatrix(ROBOT_STATE_SIZE,ROBOT_STATE_SIZE,CV_64FC1);
        for (int i = 0; i < ROBOT_STATE_SIZE; i++)
        {
            for (int j = 0; j < ROBOT_STATE_SIZE; j++)
            {
                cvmSet(Prr,i,j,cvmGet(P,i,j));
            }
            for (int j = 0; j < existing_size-ROBOT_STATE_SIZE; j++)
            {
                cvmSet(Pri,i,j,cvmGet(P,i,j+ROBOT_STATE_SIZE));
            }
        }
        
        //For each measurement
        for (int n = 0; n < n_pts; n++)
        {
            
            //Numeric Jacobian calculations to verify (uncomment if needed))
            //getGxNumeric(Gxnum,&measurements[4*n],x);
            //getGzNumeric(Gznum,&measurements[4*n],x);
            
            //Initialize the new state (the measurements are the x,y,z in body frame)
            xb->data.db[0] = measurements[3*n];
            xb->data.db[1] = measurements[3*n+1];
            xb->data.db[2] = measurements[3*n+2];
            cvMatMul(R_eb,xb,temp31);
            for (int j= 0; j < 3; j++)
                cvmSet(x,existing_size + n*3 + j,0,temp31->data.db[j] + x->data.db[j]);          //xe = R_eb*xb+T_eb, T_eb = x[0:2]
            
            point_inds.push_back(existing_size + n*3);  //Store index of new point
            
            //Find Gx
            cvmSet(Gx,0,0,1.0); //dxnew/dxr = I
            cvmSet(Gx,1,1,1.0);
            cvmSet(Gx,2,2,1.0);
            for (int j = 0; j < 3; j++)
            {
                cvMatMul(R_eb_derivs[j],xb,temp31);
                for (int i = 0; i < 3; i++)
                {
                    cvmSet(Gx,i,j+3, temp31->data.db[i]);
                }
            }
            
            //Find Gz
            cvCopy(R_eb,Gz);
            
            //Determine new covariance blocks (landmarks are independent, conditioned on the state)
            cvTranspose(Gx,GxT);
            cvMatMul(Prr,GxT,GxT);
            cvMatMul(Gx,GxT,PN1N1);

            cvTranspose(Gz,GzT);
            cvMatMul(R_pts,GzT,GzT);
            cvMatMul(Gz,GzT,temp33);
            cvAdd(PN1N1,temp33,PN1N1);

            cvMatMul(Gx,Pri,PN1i);

            cvMatMul(Gx,Prr,PN1r);

            //Add new bits (PN1N1, PN1R and PN1i, where i = other landmarks,N1 = new landmark, R = robot state)
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cvmSet( P, i+existing_size+n*3, j+existing_size+n*3, cvmGet(PN1N1,i,j) );
                }
                for (int j = 0; j < existing_size-ROBOT_STATE_SIZE; j++)
                {
                    cvmSet( P, i+existing_size+n*3, j+ROBOT_STATE_SIZE, cvmGet(PN1i,i,j) );
                    cvmSet( P, j+ROBOT_STATE_SIZE, i+existing_size+n*3, cvmGet(PN1i,i,j) );
                }

                for(int j = 0; j < ROBOT_STATE_SIZE; j++)
                {
                    cvmSet( P, i+existing_size+n*3, j, cvmGet(PN1r,i,j) );
                    cvmSet( P, j, i+existing_size+n*3, cvmGet(PN1r,i,j) );
                }
            }
            num_points++;
        }
        
        //printMatrix(Gx);
        //printMatrix(Gxnum);
        //printMatrix(Gz);
        //printMatrix(Gznum);
        
        cvReleaseMat(&Pri);
        cvReleaseMat(&PN1i);
        cvReleaseMat(&PN1N1);
        cvReleaseMat(&PN1r);
        cvReleaseMat(&PrrT);
        cvReleaseMat(&Gx);
        cvReleaseMat(&Gz);
        cvReleaseMat(&GxT);
        cvReleaseMat(&GzT);
        cvReleaseMat(&temp33);
        cvReleaseMat(&temp31);
    }
    gettimeofday(&stop, NULL);
    elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
            (start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

//Update exsting states (both curve and point landmarks, including case where no points are used)
void KalmanFilter::UpdateNCurvesAndPoints(CvMat * measurement, int n, std::vector<CvMat *> * A, vector<int> * curve_num, double * point_meas, int * point_nums, int n_pts)
{
        
        //Initialise data structures
        int curve_meas_size = n*8+3;
        int pt_meas_size = n_pts*3;
        int meas_size = pt_meas_size+curve_meas_size;
        int existing_size = P->rows;
        cvSetZero(Rot);

        CvMat * z = newMatrix(meas_size, 1, CV_64FC1);
        for (int i = 0; i < curve_meas_size; i++)
            cvmSet(z,i,0, cvmGet(measurement,i,0));
        for (int i = 0; i < pt_meas_size; i++)
            cvmSet(z,i+curve_meas_size,0, point_meas[i]);

        CvMat * H = newMatrix(meas_size, existing_size, CV_64FC1);
        CvMat * Ht = newMatrix(existing_size, meas_size, CV_64FC1);
        CvMat * K = newMatrix(existing_size, meas_size, CV_64FC1);
        CvMat * Kt = newMatrix(meas_size, existing_size, CV_64FC1);
        CvMat * Kt2 = newMatrix(meas_size, existing_size, CV_64FC1);
        CvMat * temp = newMatrix(existing_size, meas_size, CV_64FC1);
        CvMat * delx = newMatrix(existing_size, 1, CV_64FC1);
        CvMat * tempn = newMatrix(meas_size, meas_size, CV_64FC1);
        CvMat * tempn1 = newMatrix(meas_size, 1, CV_64FC1);
        CvMat * delP = newMatrix(existing_size, existing_size, CV_64FC1);
	CvMat * R = newMatrix(meas_size,meas_size,CV_64FC1);
	CvMat * S = newMatrix(meas_size,meas_size,CV_64FC1);
        CvMat * Rotderiv = newMatrix(8,8,CV_64FC1);
        CvMat * predicted_meas_pts = newMatrix(pt_meas_size+1,1,CV_64FC1); //+1 is to ensure we don't allocate non-zero
        
        //Set meas noise
        for (int i = 0; i < curve_meas_size; i++)
        {
            cvmSet(R,i,i, pow(MEAS_COV,2.0));
        }
        cvmSet(R,n*8,n*8, pow(Z_MEAS_COV,2.0));
        cvmSet(R,n*8+1,n*8+1, pow(PHI_MEAS_COV,2.0));
        cvmSet(R,n*8+2,n*8+2, pow(THETA_MEAS_COV,2.0));
        for (int i = curve_meas_size; i < meas_size; i++)
        {
            cvmSet(R,i,i, pow(PT_MEAS_COV,2.0));
        }

        cvSetZero(temp8);
        cvSetZero(temp81);
        cvSetZero(H);
        cvSetZero(Rotderiv);

        double Tx = x->data.db[0];
        double Ty = x->data.db[1];
        double phi = x->data.db[3];
        double theta = x->data.db[4];
        double psi = x->data.db[5];

        //Determine rot matrix
        for (int i = 0; i < 4; i++)
        {
            cvmSet(Rot,i,i,cos(psi));
            cvmSet(Rot,i+4,i+4,cos(psi));
            cvmSet(Rot,i+4,i,-sin(psi));
            cvmSet(Rot,i,i+4,sin(psi));
            //Rotderiv is for the terms using the partial deriv wrt psi
            cvmSet(Rotderiv,i,i,-sin(psi));
            cvmSet(Rotderiv,i+4,i+4,-sin(psi));
            cvmSet(Rotderiv,i+4,i,-cos(psi));
            cvmSet(Rotderiv,i,i+4,cos(psi));
        }
        
        //Determine H
        
        //First the curve measurement terms
        for (int k = 0; k < n; k++)
        {
            cvSetZero(temp8);
            cvSetZero(temp81);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    cvmSet( temp8, i, j, cvmGet(A->at(k),i,j));
                    cvmSet( temp8, i+4, j+4, cvmGet(A->at(k),i,j));
                }
                cvmSet(temp81,i,0, cvmGet(x,i+curve_inds[curve_num->at(k)],0));
                cvmSet(temp81,i+4,0, cvmGet(x,i+curve_inds[curve_num->at(k)]+4,0));
            }
            cvMatMul(temp8,temp81,temp81);
            for (int i = 0; i < 4; i++)
                temp81->data.db[i] -= Tx;
            for (int i = 4; i < 8; i++)
                temp81->data.db[i] -= Ty;
            cvMatMul(Rotderiv,temp81,temp81);
            cvMatMul(Rot,temp8,temp8);
            
            //Terms relating measurement to curve params
            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 8; j++)
                {
                    cvmSet( H, i+k*8, j+curve_inds[curve_num->at(k)], cvmGet(temp8,i,j));
                }
            }
            //Terms relating measurement to Tx, Ty and psi
            for (int i = 0; i < 4; i++)
            {
                cvmSet( H, i+k*8, 0, -cos(psi));
                cvmSet( H, i+k*8, 1, -sin(psi));
                cvmSet( H, i+k*8, 5, temp81->data.db[i]);
            }
            for (int i = 4; i < 8; i++)
            {
                cvmSet( H, i+k*8, 0, sin(psi));
                cvmSet( H, i+k*8, 1, -cos(psi));
                cvmSet( H, i+k*8, 5, temp81->data.db[i]);
            }

            cvSetZero(temp8);
            cvSetZero(temp81);
        }
        
        //The direct measurement of phi, theta, z
        cvmSet( H, n*8, 2, 1.0);
        cvmSet( H, 1+n*8, 3, 1.0);
        cvmSet( H, 2+n*8, 4, 1.0);
        
        //Point measurement terms
        CvMat * dzdxb = newMatrix(4,3,CV_64FC1);
        cvSetZero(temp41);
        cvSetZero(temp43);
        cvSetZero(temp31);
        CvMat * xe = newMatrix(3,1,CV_64FC1);
        CvMat * xb = newMatrix(3,1,CV_64FC1);
        CvMat * Tbe = newMatrix(3,1,CV_64FC1);
        CvMat * R_be = newMatrix(3,3,CV_64FC1);
        CvMat * R_be_derivs[3];
        R_be_derivs[0] = newMatrix(3,3,CV_64FC1);
        R_be_derivs[1] = newMatrix(3,3,CV_64FC1);
        R_be_derivs[2] = newMatrix(3,3,CV_64FC1);
        
        generate_Rbe(phi, theta, psi, R_be);
        get_Rbe_derivs(phi, theta, psi, R_be_derivs[0], R_be_derivs[1], R_be_derivs[2]);

        Tbe->data.db[0] = x->data.db[0];
        Tbe->data.db[1] = x->data.db[1];
        Tbe->data.db[2] = x->data.db[2];
        
        for (int k = 0; k < n_pts; k++)
        {
            //Get existing state corresponding to measurement, and transform to body frame too
            xe->data.db[0] = cvmGet(x,point_inds[point_nums[k]],0);
            xe->data.db[1] = cvmGet(x,point_inds[point_nums[k]]+1,0);
            xe->data.db[2] = cvmGet(x,point_inds[point_nums[k]]+2,0);
            cvSub(xe,Tbe,xe);
            cvMatMul(R_be,xe,xb);

            double xp = cvmGet(xb,0,0);
            double yp = cvmGet(xb,1,0);
            double zp = cvmGet(xb,2,0);

            //Add predicted measurement to vector too (for later)
            cvmSet(predicted_meas_pts,k*3,0,xp);
            cvmSet(predicted_meas_pts,k*3+1,0, yp);
            cvmSet(predicted_meas_pts,k*3+2,0, zp);


            //dzdr and dzdxe
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cvmSet(H,curve_meas_size+k*3+i,j,-R_be->data.db[3*i+j]);
                    cvmSet(H,curve_meas_size+k*3+i,point_inds[point_nums[k]]+j,R_be->data.db[3*i+j]);
                }
            }

            //dzPsi
            for (int j = 0; j < 3; j++)
            {
                cvMatMul(R_be_derivs[j],xe,temp31);
                for (int i = 0; i < 3; i++)
                    cvmSet(H,curve_meas_size+k*3+i,3+j,temp31->data.db[i]);
            }
        }

        cvReleaseMat(&dzdxb);
        cvReleaseMat(&xe);
        cvReleaseMat(&xb);
        cvReleaseMat(&Tbe);
        cvReleaseMat(&R_be);
        cvReleaseMat(&R_be_derivs[0]);
        cvReleaseMat(&R_be_derivs[1]);
        cvReleaseMat(&R_be_derivs[2]);

        
        //Calculate Kalman gain
        cvTranspose(H,Ht);
        cvMatMul(P,Ht,temp);
        cvMatMul(H,temp,S);
        cvAdd(S,R,S);
        cvInvert(S,tempn,CV_SVD);
        cvMatMul(Ht,tempn,K);
        cvMatMul(P,K,K);
        
        //Get numeric Jacobian to compare
        //CvMat * Hnum = newMatrix(meas_size, existing_size, CV_64FC1);
        //getHNumeric(Hnum,x, n, A, curve_num, point_nums, n_pts);
        //cvmSet(Hnum,n*8,2,1.0);
        //cvmSet(Hnum,n*8+1,3,1.0);
        //cvmSet(Hnum,n*8+2,4,1.0);
        
        //cvSub(H,Hnum,Hnum);
        //cout << cvNorm(Hnum) << endl;
        
        //Get predicted measurement
        for (int i = 0; i < n; i++)
        {
            GetPredictedMeasurement(temp81,x,A->at(i),curve_num->at(i));
            for (int j = 0; j < 8; j++)
                cvmSet(tempn1,8*i+j,0, temp81->data.db[j]);
        }
        cvmSet(tempn1,n*8,0, x->data.db[2]);
        cvmSet(tempn1,n*8+1,0, x->data.db[3]);
        cvmSet(tempn1,n*8+2,0, x->data.db[4]);
        for (int i = 0; i < pt_meas_size; i++)
        {
            cvmSet(tempn1,i+curve_meas_size,0,predicted_meas_pts->data.db[i]);
        }
        
        //Compute innovation and update state
        cvSub(z,tempn1,tempn1);
        cvMatMul(K,tempn1,delx);
        cvAdd(x,delx,x);

        while(x->data.db[3] > PI)
            x->data.db[3] -= 2*PI;
        while(x->data.db[3] < -PI)
            x->data.db[3] += 2*PI;
        while(x->data.db[4] > PI)
            x->data.db[4] -= 2*PI;
        while(x->data.db[4] < -PI)
            x->data.db[4] += 2*PI;
        while(x->data.db[5] > PI)
            x->data.db[5] -= 2*PI;
        while(x->data.db[5] < -PI)
            x->data.db[5] += 2*PI;
        
        //Update covariance matrix P
        cvTranspose(K,Kt);
        cvMatMul(S,Kt,Kt2);
        cvMatMul(K,Kt2,delP);
        cvSub(P,delP,P);
        
        //Make sure P is pos def and not NaN
        CvMat * Pt = newMatrix(P->rows,P->cols,CV_64FC1);

        cvTranspose(P,Pt);
        cvAddWeighted(P, 0.5, Pt, 0.5, 0.0, P);
        cvReleaseMat(&Pt);
        
        cvReleaseMat(&H);
        cvReleaseMat(&Ht);
        cvReleaseMat(&R);
        cvReleaseMat(&S);
        cvReleaseMat(&K);
        cvReleaseMat(&Kt);
        cvReleaseMat(&Kt2);
        cvReleaseMat(&temp);
        cvReleaseMat(&tempn);
        cvReleaseMat(&tempn1);
        cvReleaseMat(&delx);
        cvReleaseMat(&delP);
        cvReleaseMat(&Rotderiv);
        
	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

//Update existing states (only points))
void KalmanFilter::UpdatePoints(double * point_meas, int * point_nums, int n_pts)
{
	gettimeofday(&start, NULL);
        for (int i = 0; i < n_pts*3; i++)
            cout << " " << point_meas[i];
        cout << endl;
        
        //Init variables and matrices
        int meas_size = n_pts*3;
        int existing_size = P->rows;
        
        cvSetZero(Rot); 

        CvMat * z = newMatrix(meas_size, 1, CV_64FC1);
        for (int i = 0; i < meas_size; i++)
            z->data.db[i] = point_meas[i];

        CvMat * H = newMatrix(meas_size, existing_size, CV_64FC1);
        CvMat * Ht = newMatrix(existing_size, meas_size, CV_64FC1);
        CvMat * K = newMatrix(existing_size, meas_size, CV_64FC1);
        CvMat * Kt = newMatrix(meas_size, existing_size, CV_64FC1);
        CvMat * temp = newMatrix(existing_size, meas_size, CV_64FC1);
        CvMat * delx = newMatrix(existing_size, 1, CV_64FC1);
        CvMat * tempn = newMatrix(meas_size, meas_size, CV_64FC1);
        CvMat * tempn1 = newMatrix(meas_size, 1, CV_64FC1);
        CvMat * tempn3 = newMatrix(existing_size, existing_size, CV_64FC1);
        CvMat * delP = newMatrix(existing_size, existing_size, CV_64FC1);
	CvMat * R = newMatrix(meas_size,meas_size,CV_64FC1);
	CvMat * S = newMatrix(meas_size,meas_size,CV_64FC1);
        CvMat * Rotderiv = newMatrix(8,8,CV_64FC1);
        CvMat * predicted_meas_pts = newMatrix(meas_size+1,1,CV_64FC1);
        
        //Set meas noise
        for (int i = 0; i < meas_size; i++)
        {
            R->data.db[(meas_size+1)*i] = pow(PT_MEAS_COV,2.0);
        }

        cvSetZero(temp8);
        cvSetZero(temp81);
        cvSetZero(H);

        double Tx = x->data.db[0];
        double Ty = x->data.db[1];
        double Tz = x->data.db[2];
        double phi = x->data.db[3];
        double theta = x->data.db[4];
        double psi = x->data.db[5];
        
        //Determine H
        CvMat * temp31 = newMatrix(3,1,CV_64FC1);
        CvMat * xe = newMatrix(3,1,CV_64FC1);
        CvMat * xb = newMatrix(3,1,CV_64FC1);
        CvMat * Tbe = newMatrix(3,1,CV_64FC1);
        CvMat * R_be = newMatrix(3,3,CV_64FC1);
        CvMat * R_be_derivs[3];
        R_be_derivs[0] = newMatrix(3,3,CV_64FC1);
        R_be_derivs[1] = newMatrix(3,3,CV_64FC1);
        R_be_derivs[2] = newMatrix(3,3,CV_64FC1);
        generate_Rbe(phi, theta, psi, R_be);
        get_Rbe_derivs(phi, theta, psi, R_be_derivs[0], R_be_derivs[1], R_be_derivs[2]);

        Tbe->data.db[0] = x->data.db[0];
        Tbe->data.db[1] = x->data.db[1];
        Tbe->data.db[2] = x->data.db[2];


        //Point measurement terms
        for (int k = 0; k < n_pts; k++)
        {
            //Get existing state corresponding to measurement, and transform to body frame too
            xe->data.db[0] = cvmGet(x,point_inds[point_nums[k]],0);
            xe->data.db[1] = cvmGet(x,point_inds[point_nums[k]]+1,0);
            xe->data.db[2] = cvmGet(x,point_inds[point_nums[k]]+2,0);
            cvSub(xe,Tbe,xe);
            cvMatMul(R_be,xe,xb);
                
            double xp = cvmGet(xb,0,0);
            double yp = cvmGet(xb,1,0);
            double zp = cvmGet(xb,2,0);

            //Add predicted measurement to vector too (for later)
            cvmSet(predicted_meas_pts,k*3,0,xp);
            cvmSet(predicted_meas_pts,k*3+1,0, yp);
            cvmSet(predicted_meas_pts,k*3+2,0, zp);

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cvmSet(H,k*3+i,j,-R_be->data.db[3*i+j]);
                    cvmSet(H,k*3+i,point_inds[point_nums[k]]+j,R_be->data.db[3*i+j]);
                }
            }

            //dzPsi
            //cvSub(xe,Tbe,xe);
            for (int j = 0; j < 3; j++)
            {
                cvMatMul(R_be_derivs[j],xe,temp31);
                for (int i = 0; i < 3; i++)
                    cvmSet(H,k*3+i,3+j,temp31->data.db[i]);
            }
        }
        
        //Get numeric Jacobian to compare
        //CvMat * Hnum = newMatrix(meas_size, existing_size, CV_64FC1);
        //std::vector<CvMat *> * A;
        //vector<int> * curve_num;
        //getHNumeric(H,x, 0, A, curve_num, point_nums, n_pts);
        //cvSub(Hnum,H,Hnum);
        //printMatrix(Hnum);
        
        //Calculate Kalman gain
        cvTranspose(H,Ht);
        cvMatMul(P,Ht,temp);
        cvMatMul(H,temp,S);
        cvAdd(S,R,S);
        cvInvert(S,tempn,CV_SVD);
        cvMatMul(Ht,tempn,K);
        cvMatMul(P,K,K);
        

        
        //Update state
        for (int i = 0; i < meas_size; i++)
        {
            tempn1->data.db[i]=predicted_meas_pts->data.db[i];
        }
        
        cvSub(z,tempn1,tempn1);
        printMatrix(tempn1);
        cvMatMul(K,tempn1,delx);
        cvAdd(x,delx,x);

        //Scale to between -PI and PI
        while(x->data.db[3] > PI)
            x->data.db[3] -= 2*PI;
        while(x->data.db[3] < -PI)
            x->data.db[3] += 2*PI;
        while(x->data.db[4] > PI)
            x->data.db[4] -= 2*PI;
        while(x->data.db[4] < -PI)
            x->data.db[4] += 2*PI;
        while(x->data.db[5] > PI)
            x->data.db[5] -= 2*PI;
        while(x->data.db[5] < -PI)
            x->data.db[5] += 2*PI;
        
        //Update covariance matrix P
        cvTranspose(K,Kt);
        cvMatMul(S,Kt,Kt);
        cvMatMul(K,Kt,delP);
        cvSub(P,delP,P);
        
        //Make sure P is pos def
        CvMat * Pt = newMatrix(P->rows,P->cols,CV_64FC1);
        cvTranspose(P,Pt);
        cvAddWeighted(P, 0.5, Pt, 0.5, 0.0, P);
        cvReleaseMat(&Pt);
        
        cvReleaseMat(&H);
        cvReleaseMat(&Ht);
        cvReleaseMat(&R);
        cvReleaseMat(&S);
        cvReleaseMat(&K);
        cvReleaseMat(&Kt);
        cvReleaseMat(&temp);
        cvReleaseMat(&tempn);
        cvReleaseMat(&tempn1);
        cvReleaseMat(&tempn3);
        cvReleaseMat(&delx);
        cvReleaseMat(&delP);
        cvReleaseMat(&Rotderiv);
        
	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

float KalmanFilter::getTime()
{
	return elapsedTime;
}

void KalmanFilter::resetTime()
{
    elapsedTime = 0.0;
}

CvMat * KalmanFilter::getState()
{
	return x;
}

//Given a t value, obtain split matrices
void KalmanFilter::GetSplitMatrices(double t, CvMat * A1, CvMat * A2)
{
    cvSetZero(A1);
    cvSetZero(A2);
   for (int i = 0; i < 4; i++)
   {
	for (int j = 0; j <= i; j++)
		A1->data.db[i*4+j] = binom[i][j]*pow(t,j)*pow((1-t),(i-j));
	for (int j = i; j < 4; j++)
		A2->data.db[4*i+j] = binom[3-i][j-i]*pow(t,(j-i))*pow((1-t),(3-j));
   }

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

//Get predicted curve measurements based on state vector
void KalmanFilter::GetPredictedMeasurement(CvMat * z_hat, CvMat * x_current, CvMat * A, int num_curve)
{
    double Tx = x_current->data.db[0];
    double Ty = x_current->data.db[1];
    double psi = x_current->data.db[5];

    //Transform ax's to a2 curve and subtract current vehicle x
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+curve_inds[num_curve]];
    cvMatMul(A,temp41,temp41);
    for (int i = 0; i < 4; i++)
        temp81->data.db[i] = temp41->data.db[i] - Tx;


    //Transform ay's to a2 curve and subtract current vehicle y
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+curve_inds[num_curve]+4];
    cvMatMul(A,temp41,temp41);
    for (int i = 0; i < 4; i++)
        temp81->data.db[i+4] = temp41->data.db[i] - Ty;

    //Construct rotation matrix
    cvSetZero(Rot);
    for (int i = 0; i < 4; i++)
    {
        cvmSet(Rot,i,i,cos(psi));
        cvmSet(Rot,i+4,i+4,cos(psi));
        cvmSet(Rot,i+4,i,-sin(psi));
        cvmSet(Rot,i,i+4,sin(psi));
    }
    cvMatMul(Rot,temp81,temp81);

    //Copy to measurement vector
    for (int i = 0; i < 8; i++)
        z_hat->data.db[i] = temp81->data.db[i];

    
}

//Initialise new matrix and set to zero
CvMat * KalmanFilter::newMatrix(int rows, int cols, int type)
{
    CvMat * matrix = cvCreateMat(rows,cols,type);
    cvSetZero(matrix);
    return matrix;
}

bool KalmanFilter::CheckValidMeasurement(double theta, double phi, double z, int frames_since_good_measurement)
{

    if(fabs(x->data.db[2]) < 0.1)	// First time
        return true;
    
    if (fabs(phi-x->data.db[3]) > MAX(frames_since_good_measurement,3)*(PHI_MEAS_COV+cvmGet(P,3,3)))
        return false;
    if (fabs(theta-x->data.db[4]) > MAX(frames_since_good_measurement,3)*(THETA_MEAS_COV+cvmGet(P,4,4)))
        return false;
    if (fabs(z-x->data.db[2]) > MAX(frames_since_good_measurement,3)*Z_MEAS_COV)
        return false;
    
    return true;

}


//Take in Euler angles as params
//Return 3 matrices, which are the rotation matrix R_eb with each term differentiated
// with respect to phi, theta, and psi
void KalmanFilter::get_Reb_derivs(double phi, double theta, double psi, CvMat * R_eb_phi, CvMat * R_eb_theta, CvMat * R_eb_psi)
{
        cvmSet(R_eb_phi,0,0, 0.0);
        cvmSet(R_eb_phi,0,1, cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
        cvmSet(R_eb_phi,0,2, -cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi));
        cvmSet(R_eb_phi,1,0, 0.0);
        cvmSet(R_eb_phi,1,1, sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));
        cvmSet(R_eb_phi,1,2, -sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi));
        cvmSet(R_eb_phi,2,0, 0.0);
        cvmSet(R_eb_phi,2,1, cos(theta)*cos(phi));
        cvmSet(R_eb_phi,2,2, -cos(theta)*sin(phi));
        
        cvmSet(R_eb_theta,0,0, -cos(psi)*sin(theta));
        cvmSet(R_eb_theta,0,1, cos(psi)*cos(theta)*sin(phi));
        cvmSet(R_eb_theta,0,2, cos(psi)*cos(theta)*cos(phi));
        cvmSet(R_eb_theta,1,0, -sin(psi)*sin(theta));
        cvmSet(R_eb_theta,1,1, sin(psi)*cos(theta)*sin(phi));
        cvmSet(R_eb_theta,1,2, sin(psi)*cos(theta)*cos(phi));
        cvmSet(R_eb_theta,2,0, -cos(theta));
        cvmSet(R_eb_theta,2,1, -sin(theta)*sin(phi));
        cvmSet(R_eb_theta,2,2, -sin(theta)*cos(phi));
        
        cvmSet(R_eb_psi,0,0, -sin(psi)*cos(theta));
        cvmSet(R_eb_psi,0,1, -sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi));
        cvmSet(R_eb_psi,0,2, -sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));
        cvmSet(R_eb_psi,1,0, cos(psi)*cos(theta));
        cvmSet(R_eb_psi,1,1, cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi));
        cvmSet(R_eb_psi,1,2, cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
        cvmSet(R_eb_psi,2,0, 0.0);
        cvmSet(R_eb_psi,2,1, 0.0);
        cvmSet(R_eb_psi,2,2, 0.0);
}


//Take in Euler angles as params
//Return 3 matrices, which are the rotation matrix R_be with each term differentiated
// with respect to phi, theta, and psi
void KalmanFilter::get_Rbe_derivs(double phi, double theta, double psi, CvMat * R_be_phi, CvMat * R_be_theta, CvMat * R_be_psi)
{
        cvmSet(R_be_phi,0,0, 0.0);
        cvmSet(R_be_phi,1,0, cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
        cvmSet(R_be_phi,2,0, -cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi));
        cvmSet(R_be_phi,0,1, 0.0);
        cvmSet(R_be_phi,1,1, sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));
        cvmSet(R_be_phi,2,1, -sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi));
        cvmSet(R_be_phi,0,2, 0.0);
        cvmSet(R_be_phi,1,2, cos(theta)*cos(phi));
        cvmSet(R_be_phi,2,2, -cos(theta)*sin(phi));
        
        cvmSet(R_be_theta,0,0, -cos(psi)*sin(theta));
        cvmSet(R_be_theta,1,0, cos(psi)*cos(theta)*sin(phi));
        cvmSet(R_be_theta,2,0, cos(psi)*cos(theta)*cos(phi));
        cvmSet(R_be_theta,0,1, -sin(psi)*sin(theta));
        cvmSet(R_be_theta,1,1, sin(psi)*cos(theta)*sin(phi));
        cvmSet(R_be_theta,1,1, sin(psi)*cos(theta)*cos(phi));
        cvmSet(R_be_theta,0,2, -cos(theta));
        cvmSet(R_be_theta,1,2, -sin(theta)*sin(phi));
        cvmSet(R_be_theta,2,2, -sin(theta)*cos(phi));
        
        cvmSet(R_be_psi,0,0, -sin(psi)*cos(theta));
        cvmSet(R_be_psi,1,0, -sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi));
        cvmSet(R_be_psi,2,0, -sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));
        cvmSet(R_be_psi,0,1, cos(psi)*cos(theta));
        cvmSet(R_be_psi,1,1, cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi));
        cvmSet(R_be_psi,2,1, cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
        cvmSet(R_be_psi,0,2, 0.0);
        cvmSet(R_be_psi,1,2, 0.0);
        cvmSet(R_be_psi,2,2, 0.0);
}

void KalmanFilter::printMatrix(CvMat * matrix)
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

//Get numeric Jacobian for point measurements (FOR VERIFICATION ONLY!)
void KalmanFilter::getHNumeric(CvMat * H,CvMat * x, int n, std::vector<CvMat *> * A, vector<int> * curve_num, int * point_nums, int n_pts)
{
    CvMat * pt1 = newMatrix(H->rows,1,CV_64FC1);      
    CvMat * pt2 = newMatrix(H->rows,1,CV_64FC1);
    CvMat * grad = newMatrix(H->rows,1,CV_64FC1);
    CvMat * temp81 = newMatrix(8,1,CV_64FC1);
    CvMat * meas = newMatrix(3,1,CV_64FC1);
    int oop_size = 0;
    if (n)
        oop_size = 3;
        
    for (int j = 0; j < H->cols; j++)
    {
        x->data.db[j] += JAC_EPS;    
        for (int i = 0; i < n; i++)
        {
            GetPredictedMeasurement(temp81,x,A->at(i),curve_num->at(i));
            for (int k = 0; k < 8; k++)
                pt1->data.db[8*i+k]=temp81->data.db[k];
        }
        for (int i = 0; i < n_pts; i++)
        {
            predictPointMeas(meas, x, point_nums[i]);
            for (int k = 0; k < 3; k++)
                pt1->data.db[n*8+oop_size+3*i+k]=meas->data.db[k];
        }
        x->data.db[j] -= JAC_EPS;
        
        x->data.db[j] -= JAC_EPS;    
        for (int i = 0; i < n; i++)
        {
            GetPredictedMeasurement(temp81,x,A->at(i),curve_num->at(i));
            for (int j = 0; j < 8; j++)
                pt2->data.db[8*i+j]=temp81->data.db[j];
        }
        for (int i = 0; i < n_pts; i++)
        {
            predictPointMeas(meas, x, point_nums[i]);
            for (int k = 0; k < 3; k++)
                pt2->data.db[n*8+oop_size+3*i+k]=meas->data.db[k];
        }
        x->data.db[j] += JAC_EPS;
        cvSub(pt1,pt2,grad);
        for (int i = 0; i < H->rows; i++)
            cvmSet(H,i,j,grad->data.db[i]/(2.0*JAC_EPS));
    }
    cvReleaseMat(&pt1);
    cvReleaseMat(&pt2);
    cvReleaseMat(&grad);
}

//Predict point meas from state
void KalmanFilter::predictPointMeas(CvMat * meas, CvMat * x, int point_num)
{
    CvMat * R_be = newMatrix(3,3,CV_64FC1);
    CvMat * xb = newMatrix(3,1,CV_64FC1);
    CvMat * Tbe = newMatrix(3,1,CV_64FC1);
    Tbe->data.db[0] = x->data.db[0];
    Tbe->data.db[1] = x->data.db[1];
    Tbe->data.db[2] = x->data.db[2];
    generate_Rbe(x->data.db[3], x->data.db[4], x->data.db[5], R_be);
    //Get existing state corresponding to measurement, and transform to body frame too
    xb->data.db[0] = x->data.db[point_inds[point_num]];
    xb->data.db[1] = x->data.db[point_inds[point_num]+1];
    xb->data.db[2] = x->data.db[point_inds[point_num]+2];
    cvSub(xb,Tbe,xb);
    cvMatMul(R_be,xb,xb);

    //Add predicted measurement to vector too (for later)
    meas->data.db[0] = xb->data.db[0];
    meas->data.db[1] = xb->data.db[1];
    meas->data.db[2] = xb->data.db[2];
}

//Get numeric Jacobian for point initialisation wrt state (FOR VERIFICATION ONLY!)
void KalmanFilter::getGxNumeric(CvMat * Gx,double * measurement,CvMat * x)
{
    CvMat * pt1 = newMatrix(3,1,CV_64FC1);
    CvMat * pt2 = newMatrix(3,1,CV_64FC1);
    CvMat * grad = newMatrix(3,1,CV_64FC1);
    for (int j = 0; j < ROBOT_STATE_SIZE; j++)
    {
        x->data.db[j] += JAC_EPS;
        InitPointDirect(pt1,measurement,x);
        x->data.db[j] -= JAC_EPS;
        x->data.db[j] -= JAC_EPS;
        InitPointDirect(pt2,measurement,x);
        x->data.db[j] += JAC_EPS;
        cvSub(pt1,pt2,grad);
        for (int i = 0; i < 3; i++)
            cvmSet(Gx,i,j,grad->data.db[i]/(2.0*JAC_EPS));
    }
    cvReleaseMat(&pt1);
    cvReleaseMat(&pt2);
    cvReleaseMat(&grad);
}

//Get numeric Jacobian for point initialisation wrt measurement (FOR VERIFICATION ONLY!)
void KalmanFilter::getGzNumeric(CvMat * Gz,double * measurement,CvMat * x)
{    
    CvMat * pt1 = newMatrix(3,1,CV_64FC1);
    CvMat * pt2 = newMatrix(3,1,CV_64FC1);
    CvMat * grad = newMatrix(3,1,CV_64FC1);
    for (int j = 0; j < 3; j++)
    {
        measurement[j] += JAC_EPS;
        InitPointDirect(pt1,measurement,x);
        measurement[j] -= JAC_EPS;
        measurement[j] -= JAC_EPS;
        InitPointDirect(pt2,measurement,x);
        measurement[j] += JAC_EPS;
        cvSub(pt1,pt2,grad);
        for (int i = 0; i < 3; i++)
            cvmSet(Gz,i,j,grad->data.db[i]/(2.0*JAC_EPS));
    }
    cvReleaseMat(&pt1);
    cvReleaseMat(&pt2);
    cvReleaseMat(&grad);
}

//Initialise point from measurement
void KalmanFilter::InitPoint(CvMat * pt,double * measurement,CvMat * x)
{
    double xl = measurement[0];
    double yl = measurement[1];
    double xr = measurement[2];
    double yr = measurement[3];
    CvMat * R_eb = newMatrix(3,3,CV_64FC1);
    generate_Reb(x->data.db[3], x->data.db[4], x->data.db[5], R_eb);
            
    //Initialize the new state
    pt->data.db[0] = BASELINE*FX/(xl-xr);
    pt->data.db[1] = 0.5*BASELINE/(xl-xr)*(xl+xr-2.0*CX);
    pt->data.db[2] = 0.5*BASELINE/(xl-xr)*(yl+yr-2.0*CY);
    cvMatMul(R_eb,pt,pt);
    for (int j= 0; j < 3; j++)
        pt->data.db[j] += x->data.db[j];   //xe = R_eb*xb+T_eb, T_eb = x[0:2]

}

//Initialise point from measurement
void KalmanFilter::InitPointDirect(CvMat * pt,double * measurement,CvMat * x)
{
    CvMat * R_eb = newMatrix(3,3,CV_64FC1);
    generate_Reb(x->data.db[3], x->data.db[4], x->data.db[5], R_eb);
            
    //Initialize the new state
    pt->data.db[0] = measurement[0];
    pt->data.db[1] = measurement[1];
    pt->data.db[2] = measurement[2];
    cvMatMul(R_eb,pt,pt);
    for (int j= 0; j < 3; j++)
        pt->data.db[j] += x->data.db[j];   //xe = R_eb*xb+T_eb, T_eb = x[0:2]

}

//Get numeric Jacobian for curve initialisation wrt state (FOR VERIFICATION ONLY!)
void KalmanFilter::getGxCurveNumeric(CvMat * Gx,CvMat * z,CvMat * x, CvMat * A)
{
    CvMat * pt1 = newMatrix(8,1,CV_64FC1);
    CvMat * pt2 = newMatrix(8,1,CV_64FC1);
    CvMat * grad = newMatrix(8,1,CV_64FC1);
    for (int j = 0; j < ROBOT_STATE_SIZE; j++)
    {
        x->data.db[j] += JAC_EPS;
        InitCurve(pt1,z,x,A);
        x->data.db[j] -= JAC_EPS;
        x->data.db[j] -= JAC_EPS;
        InitCurve(pt2,z,x,A);
        x->data.db[j] += JAC_EPS;
        cvSub(pt1,pt2,grad);
        for (int i = 0; i < 8; i++)
            cvmSet(Gx,i,j,grad->data.db[i]/(2.0*JAC_EPS));
    }
    cvReleaseMat(&pt1);
    cvReleaseMat(&pt2);
    cvReleaseMat(&grad);
}

//Get numeric Jacobian for curve initialisation wrt measurement (FOR VERIFICATION ONLY!)
void KalmanFilter::getGzCurveNumeric(CvMat * Gz,CvMat * z,CvMat * x, CvMat * A)
{
    CvMat * pt1 = newMatrix(8,1,CV_64FC1);
    CvMat * pt2 = newMatrix(8,1,CV_64FC1);
    CvMat * grad = newMatrix(8,1,CV_64FC1);
    for (int j = 0; j < z->rows; j++)
    {
        z->data.db[j] += JAC_EPS;
        InitCurve(pt1,z,x,A);
        z->data.db[j] -= JAC_EPS;
        z->data.db[j] -= JAC_EPS;
        InitCurve(pt2,z,x,A);
        z->data.db[j] += JAC_EPS;
        cvSub(pt1,pt2,grad);
        for (int i = 0; i < 8; i++)
            cvmSet(Gz,i,j,grad->data.db[i]/(2.0*JAC_EPS));
    }
    cvReleaseMat(&pt1);
    cvReleaseMat(&pt2);
    cvReleaseMat(&grad);
}

//Initialise curve from measurement
void KalmanFilter::InitCurve(CvMat * curve, CvMat * z, CvMat * x, CvMat * A)
{
    CvMat * Ainv = newMatrix(4,4,CV_64FC1);
    CvMat * zx = newMatrix(4,1,CV_64FC1);
    CvMat * zy = newMatrix(4,1,CV_64FC1);
    CvMat * Hinv = newMatrix(8,8,CV_64FC1);
    CvMat * R = newMatrix(8,8,CV_64FC1);
    double psi = x->data.db[5];
    
    for (int i = 0; i < 4; i++)
        {
            cvmSet(R,i,i,cos(psi));
            cvmSet(R,i+4,i+4,cos(psi));
            cvmSet(R,i+4,i,sin(psi));
            cvmSet(R,i,i+4,-sin(psi));
        }
    
    for (int i = 0; i < 4; i++)
        {
            cvmSet( zx, i, 0, z->data.db[i]);
            cvmSet( zy, i, 0, z->data.db[i+4]);

        }

        cvInvert(A,Ainv,CV_SVD);
        cvMatMul(Ainv,zx,Ainvzx);
        cvMatMul(Ainv,zy,Ainvzy);
        
        //Determine new H and add new curve
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                cvmSet( Hinv, i, j, Ainv->data.db[4*i+j]);
                cvmSet( Hinv, i+4, j+4, Ainv->data.db[4*i+j]);
            }

        }
        cvMatMul(R,z,curve);
        for (int i = 0; i < 4; i++)
            curve->data.db[i] += x->data.db[0];
        for (int i = 4; i < 8; i++)
            curve->data.db[i] += x->data.db[1];
        cvMatMul(Hinv,curve,curve);
}