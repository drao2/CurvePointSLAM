#include "kalmanClass.h"
#include "displayClass.h"

using namespace std;

KalmanFilter::KalmanFilter()
{
        Ainv = newMatrix(4,4,CV_64FC1);
        Ainvzx = newMatrix(4,1,CV_64FC1);
        Ainvzy = newMatrix(4,1,CV_64FC1);
        zx = newMatrix(4,1,CV_64FC1);
        zy = newMatrix(4,1,CV_64FC1);


	A1 = newMatrix(4,4,CV_64FC1);
	A2 = newMatrix(4,4,CV_64FC1);
	B1 = newMatrix(4,4,CV_64FC1);
	B2 = newMatrix(4,4,CV_64FC1);
	H = newMatrix(8,8,CV_64FC1);
	H1 = newMatrix(8,8+ROBOT_STATE_SIZE,CV_64FC1);
	H1t = newMatrix(8+ROBOT_STATE_SIZE,8,CV_64FC1);
	Hinv = newMatrix(8,8,CV_64FC1);
	K1 = newMatrix(8+ROBOT_STATE_SIZE,8,CV_64FC1);
	delP1 = newMatrix(8+ROBOT_STATE_SIZE,8+ROBOT_STATE_SIZE,CV_64FC1);
	R1 = newMatrix(8,8,CV_64FC1);
	R_pts = newMatrix(4,4,CV_64FC1);
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
	temp19 = newMatrix(16+ROBOT_STATE_SIZE,16+ROBOT_STATE_SIZE,CV_64FC1);
	temp16 = newMatrix(16,16,CV_64FC1);
	temp161 = newMatrix(16,1,CV_64FC1);
	temp1916 = newMatrix(16+ROBOT_STATE_SIZE,16,CV_64FC1);
	temp11 = newMatrix(8+ROBOT_STATE_SIZE,8+ROBOT_STATE_SIZE,CV_64FC1);
	temp8 = newMatrix(8,8,CV_64FC1);
	temp81 = newMatrix(8,1,CV_64FC1);
	temp118 = newMatrix(8+ROBOT_STATE_SIZE,8,CV_64FC1);
	temp4 = newMatrix(4,4,CV_64FC1);
	temp41 = newMatrix(4,1,CV_64FC1);
        Rot = newMatrix(8,8,CV_64FC1);
        Rotderiv = newMatrix(8,8,CV_64FC1);
        
        
        
        Reb = newMatrix(3,3,CV_64FC1);
        Rebt = newMatrix(3,3,CV_64FC1);
        Reb_phi = newMatrix(3,3,CV_64FC1);
        Reb_theta = newMatrix(3,3,CV_64FC1);
        Reb_psi = newMatrix(3,3,CV_64FC1);
        E = newMatrix(3,3,CV_64FC1);
        Et = newMatrix(3,3,CV_64FC1);
        E_phi = newMatrix(3,3,CV_64FC1);
        E_theta = newMatrix(3,3,CV_64FC1);
        
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

        temp38 = newMatrix(ROBOT_STATE_SIZE,8,CV_64FC1);
        temp83 = newMatrix(8,ROBOT_STATE_SIZE,CV_64FC1);
        temp88 = newMatrix(8,8,CV_64FC1);
        Prr = newMatrix(ROBOT_STATE_SIZE,ROBOT_STATE_SIZE,CV_64FC1);


        // R is diagonal matrix with measurement covariance
        for (int i = 0; i < 8; i++)
        {
            R1->data.db[9*i] = MEAS_COV;
        }
        for (int i = 0; i < 4; i++)
        {
            R_pts->data.db[5*i] = PT_MEAS_COV;
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
    cvReleaseMat(&x);
    cvReleaseMat(&P);
    cvReleaseMat(&xcurrent8);
    cvReleaseMat(&Pcurrent8);

    cvReleaseMat(&Fpose);
    cvReleaseMat(&Qpose);

}


void KalmanFilter::PredictKF(CvMat * R_predict, CvMat * t_predict)
{
    CvMat * Pri = cvCreateMat(ROBOT_STATE_SIZE,num_curves*8,CV_64FC1);

    cvSetZero(Prr);
    cvSetZero(Pri);
    
    double phi = x->data.db[3];
    double theta = x->data.db[4];
    double psi = x->data.db[5];
    
        

                generate_Reb(phi, theta, psi, Reb);
        
            //Update translation
            cvMatMul(Reb,t_predict,t_predict);
            x->data.db[0] += t_predict->data.db[0];
            x->data.db[1] += t_predict->data.db[1];
            x->data.db[2] += t_predict->data.db[2];

            //Update rotation
            cvMatMul(Reb,R_predict,Reb);
            
            x->data.db[4] = asin(-Reb->data.db[6]);
            x->data.db[3] = atan2(Reb->data.db[7]/cos(x->data.db[4]),Reb->data.db[8]/cos(x->data.db[4]));
            x->data.db[5] = atan2(Reb->data.db[3]/cos(x->data.db[4]),Reb->data.db[0]/cos(x->data.db[4]));

        
    
    
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
            Qpose->data.db[ROBOT_STATE_SIZE*i+j] = pow(DT,3.0)*temp33->data.db[3*i+j];
            Qpose->data.db[ROBOT_STATE_SIZE*i+j+6] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[ROBOT_STATE_SIZE*(j+6)+i] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[ROBOT_STATE_SIZE*(i+6)+(j+6)] = pow(DT,1.0)*temp33->data.db[3*i+j];
        }
    }
    
    cvSetZero(temp33);
    temp33->data.db[0] = pow(WX_COV,2.0);
    temp33->data.db[4] = pow(WY_COV,2.0);
    temp33->data.db[8] = pow(WZ_COV,2.0);
    
    //Cov(Psi,omega) = ...
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Qpose->data.db[12*(i+3)+(j+3)] = pow(DT,3.0)*temp33->data.db[3*i+j];
            Qpose->data.db[12*(i+3)+(j+9)] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[12*(j+9)+(i+3)] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[12*(i+9)+(j+9)] = pow(DT,1.0)*temp33->data.db[3*i+j];
        }
    }
    
    //Qpose->data.db[0] += 0.2*DT;
    //Qpose->data.db[13] += 0.2*DT;
    //Qpose->data.db[65] += 0.005*DT;
    //Qpose->data.db[0] += 0.1*DT;
    //Qpose->data.db[13] += 0.01*DT;
    //Qpose->data.db[65] += 0.005*DT;

    //Get current covariance parts
    for (int i = 0; i < ROBOT_STATE_SIZE; i++)
    {
        for (int j = 0; j < ROBOT_STATE_SIZE; j++)
        {
            cvmSet(Prr,i,j,cvmGet(P,i,j));
        }
        for (int j = ROBOT_STATE_SIZE; j < num_curves*8+ROBOT_STATE_SIZE; j++)
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
        for (int j = ROBOT_STATE_SIZE; j < num_curves*8+ROBOT_STATE_SIZE; j++)
        {
            cvmSet(P,i,j,cvmGet(Pri,i,j-ROBOT_STATE_SIZE));
            cvmSet(P,j,i,cvmGet(Pri,i,j-ROBOT_STATE_SIZE));
        }

    }
    
    //Predict state    
    //x->data.db[0] += x->data.db[6]*DT;
    //x->data.db[1] += x->data.db[7]*DT;
    //x->data.db[2] += x->data.db[8]*DT;
    
    //x->data.db[3] += x->data.db[9]*DT;
    //x->data.db[4] += x->data.db[10]*DT;
    //x->data.db[5] += x->data.db[11]*DT;

    cvReleaseMat(&Pri);
}

void KalmanFilter::PredictKF()
{
    CvMat * Pri = cvCreateMat(ROBOT_STATE_SIZE,num_curves*8,CV_64FC1);

    cvSetZero(Prr);
    cvSetZero(Pri);
    
    //cout << "X = " << x->data.db[0] << endl;
    //cout << "Y = " << x->data.db[1] << endl;
    //cout << "Wz = " << x->data.db[11] << endl;
    
    double phi = 0.0;//x->data.db[3];
    double theta = 0.0;//x->data.db[4];
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
            Qpose->data.db[ROBOT_STATE_SIZE*i+j] = pow(DT,3.0)*temp33->data.db[3*i+j];
            Qpose->data.db[ROBOT_STATE_SIZE*i+j+6] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[ROBOT_STATE_SIZE*(j+6)+i] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[ROBOT_STATE_SIZE*(i+6)+(j+6)] = pow(DT,1.0)*temp33->data.db[3*i+j];
        }
    }
    
    cvSetZero(temp33);
    temp33->data.db[0] = pow(WX_COV,2.0);
    temp33->data.db[4] = pow(WY_COV,2.0);
    temp33->data.db[8] = pow(WZ_COV,2.0);
    
    //Cov(Psi,omega) = ...
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Qpose->data.db[12*(i+3)+(j+3)] = pow(DT,3.0)*temp33->data.db[3*i+j];
            Qpose->data.db[12*(i+3)+(j+9)] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[12*(j+9)+(i+3)] = pow(DT,2.0)*temp33->data.db[3*i+j];
            Qpose->data.db[12*(i+9)+(j+9)] = pow(DT,1.0)*temp33->data.db[3*i+j];
        }
    }
    
    //Qpose->data.db[0] += 0.2*DT;
    //Qpose->data.db[13] += 0.2*DT;
    //Qpose->data.db[65] += 0.005*DT;
    //Qpose->data.db[0] += 0.1*DT;
    //Qpose->data.db[13] += 0.01*DT;
    //Qpose->data.db[65] += 0.005*DT;

    //Get current covariance parts
    for (int i = 0; i < ROBOT_STATE_SIZE; i++)
    {
        for (int j = 0; j < ROBOT_STATE_SIZE; j++)
        {
            cvmSet(Prr,i,j,cvmGet(P,i,j));
        }
        for (int j = ROBOT_STATE_SIZE; j < num_curves*8+ROBOT_STATE_SIZE; j++)
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
        for (int j = ROBOT_STATE_SIZE; j < num_curves*8+ROBOT_STATE_SIZE; j++)
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


void KalmanFilter::AddFirstStates(CvMat * measurement)
{
    cvSetZero(P);
    cvSetZero(x);
        for (int i = 0; i < 16; i++)
        {
            cvmSet(P, i+ROBOT_STATE_SIZE, i+ROBOT_STATE_SIZE, R1->data.db[9*(i%8)]);
            x->data.db[i+ROBOT_STATE_SIZE] = measurement->data.db[i];
        }
    //z, phi and theta
    x->data.db[2] = measurement->data.db[16];    
    x->data.db[3] = measurement->data.db[17];    
    x->data.db[4] = measurement->data.db[18];
    x->data.db[6] = VREAL;
    
            cvmSet(P, 2, 2, pow(Z_MEAS_COV,2.0));
            cvmSet(P, 3, 3, pow(PHI_MEAS_COV,2.0));
            cvmSet(P, 4, 4, pow(THETA_MEAS_COV,2.0));
            cvmSet(P, 0, 0, 0.0000001);
            cvmSet(P, 1, 1, 0.0000001);
            cvmSet(P, 5, 5, 0.0000001);
            cvmSet(P, 6, 6, 0.1);
            cvmSet(P, 7, 7, 0.01);
            cvmSet(P, 8, 8, 0.01);
            cvmSet(P, 9, 9, 0.1);
            cvmSet(P, 10, 10, 0.1);
            cvmSet(P, 11, 11, 0.1);
    
        num_curves = 2;
}


void KalmanFilter::AddNewCurve(CvMat * z, CvMat * A)
{
    
        cout << "ADDING NEW CURVE\n";
        cvShowImage("Pold",P);
	gettimeofday(&start, NULL);
        
        double Tx = x->data.db[0];
        double Ty = x->data.db[1];
        double psi = x->data.db[5];
        cvSetZero(H);
        cvSetZero(temp8);
        cvSetZero(temp81);
        cvSetZero(Rot);
        cvSetZero(Rotderiv);
        
        //Determine rot matrix
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
        
        CvMat * PN1N1 = newMatrix(8,8,CV_64FC1);
        CvMat * PN1r = newMatrix(8,ROBOT_STATE_SIZE,CV_64FC1);
        CvMat * PN1i = newMatrix(8,(num_curves-1)*8,CV_64FC1); //For curve terms only
        CvMat * Pri = newMatrix(ROBOT_STATE_SIZE,(num_curves-1)*8,CV_64FC1); //For curve terms only
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
        int existing_size = Pcopy->rows;
        cvReleaseMat(&P);
        cvReleaseMat(&x);
        P = newMatrix(existing_size+8,existing_size+8,CV_64FC1);
        x = newMatrix(existing_size+8,1,CV_64FC1);
        
        cvShowImage("Pold1",P);
        //Find Prr and Pri first
        for (int i = 0; i < ROBOT_STATE_SIZE; i++)
        {
            for (int j = 0; j < ROBOT_STATE_SIZE; j++)
                cvmSet(Prr,i,j,cvmGet(P,i,j));
            for (int j = 0; j < (num_curves-1)*8; j++)
                cvmSet(Pri,i,j,cvmGet(P,i,j+ROBOT_STATE_SIZE));
        }

        //Changing the state vector and covariance is tricky, since we need to insert it in the middle
        //The robot state and existing curves are in the same place:
        for (int i = 0; i < (num_curves-1)*8+ROBOT_STATE_SIZE; i++)
        {
            for (int j = 0; j < (num_curves-1)*8+ROBOT_STATE_SIZE; j++)
            {
                cvmSet(P,i,j,cvmGet(Pcopy,i,j));
            }
            x->data.db[i] = xcopy->data.db[i];
        }
        
        cvShowImage("Pold2",P);
        if(num_points)
        {
            //The existing point landmarks need to be moved down (3 blocks, Pri,Pir, Pii):
            for (int i = 0; i < ROBOT_STATE_SIZE; i++)
            {
                for (int j = (num_curves-1)*8+ROBOT_STATE_SIZE; j < existing_size; j++)
                    cvmSet(P,i,j+8,cvmGet(Pcopy,i,j));
            }
            
        cvShowImage("Pold3",P);
            for (int j = 0; j < ROBOT_STATE_SIZE; j++)
            {
                for (int i = (num_curves-1)*8+ROBOT_STATE_SIZE; i < existing_size; i++)
                    cvmSet(P,i+8,j,cvmGet(Pcopy,i,j));
            }
        
        cvShowImage("Pold4",P);
            for (int i = (num_curves-1)*8+ROBOT_STATE_SIZE; i < existing_size; i++)
            {
                for (int j = (num_curves-1)*8+ROBOT_STATE_SIZE; j < existing_size; j++)
                    cvmSet(P,i+8,j+8,cvmGet(Pcopy,i,j));
            }
        
        cvShowImage("Pold5",P);
        }

        cvReleaseMat(&xcopy);
        cvReleaseMat(&Pcopy);



        cvSetZero(Pcurrent8);
        cvSetZero(xcurrent8);
        cvSetZero(H);
        cvSetZero(Hinv);



        
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
        cvMatMul(Hinv,temp81,xcurrent8);



        
        cvMatMul(Ainv,ones,temp41);



        //Figure out covariances Gz and Gx
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


        //Expand array ROI and add new bits

        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                cvmSet( P, i+(num_curves-1)*8+ROBOT_STATE_SIZE, j+(num_curves-1)*8+ROBOT_STATE_SIZE, PN1N1->data.db[8*i+j] );
            }
            
                cvShowImage("Pnew1",P);
            for (int j = 0; j < (num_curves-1)*8; j++)
            {
                cvmSet( P, i+(num_curves-1)*8+ROBOT_STATE_SIZE, j+ROBOT_STATE_SIZE, cvmGet(PN1i,i,j) );
                cvmSet( P, j+ROBOT_STATE_SIZE, i+(num_curves-1)*8+ROBOT_STATE_SIZE, cvmGet(PN1i,i,j) );
            }
            x->data.db[(num_curves-1)*8+i+ROBOT_STATE_SIZE] = xcurrent8->data.db[i];
                cvShowImage("Pnew2",P);

            for(int j = 0; j < ROBOT_STATE_SIZE; j++)
            {
                cvmSet( P, i+(num_curves-1)*8+ROBOT_STATE_SIZE, j, cvmGet(PN1r,i,j) );
                cvmSet( P, j, i+(num_curves-1)*8+ROBOT_STATE_SIZE, cvmGet(PN1r,i,j) );
            }
                cvShowImage("Pnew3",P);
        }
        
        if (num_points) //Only worry about the point terms if num_points != 0
        {
            CvMat * PN1i_pts = newMatrix(8,num_points*3,CV_64FC1);  //For Point terms only
            CvMat * Pri_pts = newMatrix(ROBOT_STATE_SIZE,num_points*3,CV_64FC1);    //For point terms only
            for (int i = 0; i < ROBOT_STATE_SIZE; i++)
            {
                for (int j = 0; j < num_points*3; j++)
                    cvmSet(Pri_pts,i,j,cvmGet(P,i,j+ROBOT_STATE_SIZE+(num_curves-1)*8));
            }
            //Add the points-curves covariance block
            cvMatMul(Gx,Pri_pts,PN1i_pts);
            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < num_points*3; j++)
                {
                    cvmSet( P, i+(num_curves-1)*8+ROBOT_STATE_SIZE, j+ROBOT_STATE_SIZE+num_curves*8, cvmGet(PN1i_pts,i,j) );
                    cvmSet( P, j+ROBOT_STATE_SIZE+num_curves*8, i+(num_curves-1)*8+ROBOT_STATE_SIZE, cvmGet(PN1i_pts,i,j) );
                }
            }
        }
        cvShowImage("Pnew",P);
        cvWaitKey(0);

        cvReleaseMat(&Pri);
        cvReleaseMat(&PN1i);
        cvReleaseMat(&Pcopy);
        cvReleaseMat(&xcopy);
        cvReleaseMat(&PN1N1);
        cvReleaseMat(&PN1r);
        cvReleaseMat(&ones);
        cvReleaseMat(&temp58);
        cvReleaseMat(&temp5);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);


}

void KalmanFilter::AddNewPoints(double * measurements, int n_pts)
{
    gettimeofday(&start, NULL);
    if (n_pts)
    {
        
        //cout << "x: " << endl;
        //printMatrix(x);
        //cout << "P: " << endl;
        //printMatrix(P);
        //cout << "ADDING NEW POINTS\n";
        //cvShowImage("Pold",P);
        
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

        //Set new landmark states, and find Gx and Gz, and fill in covariance blocks, all in one
        //Gx is not the same for all landmarks, since the angular terms are coupled with xb!
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
        CvMat * GxT = newMatrix(ROBOT_STATE_SIZE,3,CV_64FC1);
        CvMat * Gz = newMatrix(3,4,CV_64FC1);
        CvMat * GzT = newMatrix(4,3,CV_64FC1);
        
        //Find Prr and Pri, to use later in calculating the covariances
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
            double xl = measurements[4*n];
            double yl = measurements[4*n+1];
            double xr = measurements[4*n+2];
            double yr = measurements[4*n+3];
            
            //Initialize the new state
            xb->data.db[0] = BASELINE*FX/(xl-xr);
            xb->data.db[1] = 0.5*BASELINE/(xl-xr)*(xl+xr-2.0*CX);
            xb->data.db[2] = 0.5*BASELINE/(xl-xr)*(yl+yr-2.0*CY);
            cvMatMul(R_eb,xb,temp31);
            for (int j= 0; j < 3; j++)
                x->data.db[existing_size + n*3 + j] = temp31->data.db[j] + x->data.db[j];   //xe = R_eb*xb+T_eb, T_eb = x[0:2]
            
            //Find Gx
            cvmSet(Gx,0,0,1.0); //dxnew/dxr = I
            cvmSet(Gx,1,1,1.0);
            cvmSet(Gx,2,2,1.0);
            for (int j = 0; j < 3; j++)
            {
                cvMatMul(R_eb_derivs[j],xb,temp31);
                for (int i = 0; i < 3; i++)
                {
                    //cvmSet(Gx,i,j, R_eb->data.db[3*i+j]);
                    cvmSet(Gx,i,j+3, temp31->data.db[i]);
                }
            }
            //Find Gz
            cvmSet(Gz,0,0,-BASELINE*FX/((xl-xr)*(xl-xr)));
            cvmSet(Gz,1,0,0.5*BASELINE/(xl-xr)*(1.0 - (xl+xr-2*CX)/(xl-xr)));
            cvmSet(Gz,2,0,-0.5*BASELINE/((xl-xr)*(xl-xr))*(yl+yr-2*CY));
            cvmSet(Gz,0,2,BASELINE*FX/((xl-xr)*(xl-xr)));
            cvmSet(Gz,1,2,0.5*BASELINE/(xl-xr)*(1.0 + (xl+xr-2*CX)/(xl-xr)));
            cvmSet(Gz,2,2,0.5*BASELINE/((xl-xr)*(xl-xr))*(yl+yr-2*CY));
            cvmSet(Gz,0,1,0.0);
            cvmSet(Gz,1,1,0.0);
            cvmSet(Gz,2,1,0.5*BASELINE/(xl-xr));
            cvmSet(Gz,0,3,0.0);
            cvmSet(Gz,1,3,0.0);
            cvmSet(Gz,2,3,0.5*BASELINE/(xl-xr));
            cvMatMul(R_eb,Gz,Gz);
            
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


            //Add new bits (PN1N1, PN1R and PN1i, where i = other landmarks)

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cvmSet( P, i+existing_size+n*3, j+existing_size+n*3, PN1N1->data.db[3*i+j] );
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


        
        cvReleaseMat(&Pri);
        cvReleaseMat(&PN1i);
        cvReleaseMat(&Pcopy);
        cvReleaseMat(&xcopy);
        cvReleaseMat(&PN1N1);
        cvReleaseMat(&PN1r);
        cvReleaseMat(&PrrT);
        cvReleaseMat(&Gx);
        cvReleaseMat(&Gz);
        cvReleaseMat(&GxT);
        cvReleaseMat(&GzT);
        cvReleaseMat(&temp33);
        cvReleaseMat(&temp31);
        
        
        //cout << "x_new: " << endl;
        //printMatrix(x);
        //cout << "P_new: " << endl;
        //printMatrix(P);
        
        //cvShowImage("Pnew",P);
        //cvWaitKey(0);
    }
	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

void KalmanFilter::UpdateNCurvesAndPoints(CvMat * measurement, int n, std::vector<CvMat *> * A, vector<int> * curve_num, double * point_meas, int * point_nums, int n_pts)
{
	gettimeofday(&start, NULL);
        //cout << "UPDATING LANDMARKS\n";
        //cvShowImage("Pold",P);
        
        cvSetZero(Rot);

        CvMat * z = newMatrix(n*8+3, 1, CV_64FC1);
        for (int i = 0; i < z->rows; i++)
            z->data.db[i] = measurement->data.db[i];

        int existing_size = P->rows;
        CvMat * H = newMatrix(n*8+3, existing_size, CV_64FC1);
        CvMat * Ht = newMatrix(existing_size, n*8+3, CV_64FC1);
        CvMat * K = newMatrix(existing_size, n*8+3, CV_64FC1);
        CvMat * Kt = newMatrix(n*8+3, existing_size, CV_64FC1);
        CvMat * temp = newMatrix(existing_size, n*8+3, CV_64FC1);
        CvMat * delx = newMatrix(existing_size, 1, CV_64FC1);
        CvMat * tempn = newMatrix(n*8+3, n*8+3, CV_64FC1);
        CvMat * tempn1 = newMatrix(n*8+3, 1, CV_64FC1);
        CvMat * tempn3 = newMatrix(existing_size, existing_size, CV_64FC1);
        CvMat * delP = newMatrix(existing_size, existing_size, CV_64FC1);
	CvMat * R = newMatrix(n*8+3,n*8+3,CV_64FC1);
	CvMat * S = newMatrix(n*8+3,n*8+3,CV_64FC1);
        CvMat * Rotderiv = newMatrix(8,8,CV_64FC1);
        
        for (int i = 0; i < z->rows; i++)
        {
            R->data.db[(n*8+4)*i] = MEAS_COV;
        }
            R->data.db[(n*8+4)*(n*8)] = pow(Z_MEAS_COV,2.0);
            R->data.db[(n*8+4)*(n*8+1)] = pow(PHI_MEAS_COV,2.0);
            R->data.db[(n*8+4)*(n*8+2)] = pow(THETA_MEAS_COV,2.0);

        cvSetZero(temp8);
        cvSetZero(temp81);
        cvSetZero(temp16);
        cvSetZero(temp161);
        cvSetZero(H);

        
        cvSetZero(Rotderiv);

        double Tx = x->data.db[0];
        double Ty = x->data.db[1];
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
        for (int k = 0; k < n; k++)
        {
            cvSetZero(temp8);
            cvSetZero(temp81);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    cvmSet( temp8, i, j, A->at(k)->data.db[4*i+j]);
                    cvmSet( temp8, i+4, j+4, A->at(k)->data.db[4*i+j]);
                }
                temp81->data.db[i] = x->data.db[i+curve_num->at(k)*8+ROBOT_STATE_SIZE];
                temp81->data.db[i+4] = x->data.db[i+curve_num->at(k)*8+ROBOT_STATE_SIZE+4];
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
                        cvmSet( H, i+k*8, j+curve_num->at(k)*8+ROBOT_STATE_SIZE, temp8->data.db[8*i+j]);
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
      
        cvmSet( H, n*8, 2, 1.0);
        cvmSet( H, 1+n*8, 3, 1.0);
        cvmSet( H, 2+n*8, 4, 1.0);
        
        for (int i = 0; i < H->rows; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                cvmSet( H, i, j+6, cvmGet( H, i, j)*DT);
            }
            
        }

        cvTranspose(H,Ht);

        //Calculate Kalman gain
        cvMatMul(P,Ht,temp);
        cvMatMul(H,temp,S);
        cvAdd(S,R,S);
        cvInvert(S,tempn,CV_LU);
        cvMatMul(Ht,tempn,K);
        cvMatMul(P,K,K);
        
        //Update state
        for (int i = 0; i < n; i++)
        {
            GetPredictedMeasurement(temp81,x,A->at(i),curve_num->at(i));
            for (int j = 0; j < 8; j++)
                tempn1->data.db[8*i+j]=temp81->data.db[j];
        }
        tempn1->data.db[n*8] = x->data.db[2];
        tempn1->data.db[n*8+1] = x->data.db[3];
        tempn1->data.db[n*8+2] = x->data.db[4];

        //cout << "Measurement:\n";
        //printMatrix(z);
        //cout << "Predicted Measurement:\n";
        //printMatrix(tempn1);
        cvSub(z,tempn1,tempn1);

        //cout << "Meas error:\n";
        //printMatrix(tempn1);
        cvMatMul(K,tempn1,delx);

        //cout << "H:\n";
        //printMatrix(H);
        //cout << "K:\n";
        //printMatrix(K);
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
        cvMatMul(S,Kt,Kt);
        cvMatMul(K,Kt,delP);
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
        cvReleaseMat(&temp);
        cvReleaseMat(&tempn);
        cvReleaseMat(&tempn1);
        cvReleaseMat(&tempn3);
        cvReleaseMat(&delx);
        cvReleaseMat(&delP);
        cvReleaseMat(&Rotderiv);
        cvReleaseMat(&Pt);
        cout << num_curves << " " << num_points << " " << x->rows << " " << P->rows << endl;
        //cout << "x:\n" << endl;
        //printMatrix(x);
        
	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
        
        
        //cvShowImage("Pnew",P);
        //cvWaitKey(0);
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


void KalmanFilter::GetPredictedMeasurement(CvMat * z_hat, CvMat * x_current, CvMat * A, CvMat * B, int num_curve1, int num_curve2)
{
    double Tx = x_current->data.db[0];
    double Ty = x_current->data.db[1];
    double psi = x_current->data.db[5];

    //Transform ax's to a2 curve and subtract current vehicle x
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+ROBOT_STATE_SIZE];
    cvMatMul(A,temp41,temp41);
    for (int i = 0; i < 4; i++)
        temp81->data.db[i] = temp41->data.db[i] - Tx;


    //Transform ay's to a2 curve and subtract current vehicle y
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+ROBOT_STATE_SIZE+4];
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



    //Transform bx's to b1 curve and subtract current vehicle x
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+ROBOT_STATE_SIZE+8];
    cvMatMul(B,temp41,temp41);
    for (int i = 0; i < 4; i++)
        temp81->data.db[i] = temp41->data.db[i] - Tx;


    //Transform by's to b1 curve and subtract current vehicle y
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+ROBOT_STATE_SIZE+12];
    cvMatMul(B,temp41,temp41);
    for (int i = 0; i < 4; i++)
        temp81->data.db[i+4] = temp41->data.db[i] - Ty;

    cvMatMul(Rot,temp81,temp81);

    //Copy to measurement vector
    for (int i = 0; i < 8; i++)
        z_hat->data.db[i+8] = temp81->data.db[i];

    

}

void KalmanFilter::GetPredictedMeasurement(CvMat * z_hat, CvMat * x_current, CvMat * A, int num_curve)
{
    double Tx = x_current->data.db[0];
    double Ty = x_current->data.db[1];
    double psi = x_current->data.db[5];

    //Transform ax's to a2 curve and subtract current vehicle x
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+num_curve*8+ROBOT_STATE_SIZE];
    cvMatMul(A,temp41,temp41);
    for (int i = 0; i < 4; i++)
        temp81->data.db[i] = temp41->data.db[i] - Tx;


    //Transform ay's to a2 curve and subtract current vehicle y
    for (int i = 0; i < 4; i++)
        temp41->data.db[i] = x_current->data.db[i+num_curve*8+ROBOT_STATE_SIZE+4];
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


void KalmanFilter::get_Reb_derivs(double phi, double theta, double psi, CvMat * R_eb_phi, CvMat * R_eb_theta, CvMat * R_eb_psi)
{
        R_eb_phi->data.db[0] = 0.0;
        R_eb_phi->data.db[1] = cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
        R_eb_phi->data.db[2] = -cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
        R_eb_phi->data.db[3] = 0.0;
        R_eb_phi->data.db[4] = sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
        R_eb_phi->data.db[5] = -sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi);
        R_eb_phi->data.db[6] = 0.0;
        R_eb_phi->data.db[7] = cos(theta)*cos(phi);
        R_eb_phi->data.db[8] = -cos(theta)*sin(phi);
        
        R_eb_theta->data.db[0] = -cos(psi)*sin(theta);
        R_eb_theta->data.db[1] = cos(psi)*cos(theta)*sin(phi);
        R_eb_theta->data.db[2] = cos(psi)*cos(theta)*cos(phi);
        R_eb_theta->data.db[3] = -sin(psi)*sin(theta);
        R_eb_theta->data.db[4] = sin(psi)*cos(theta)*sin(phi);
        R_eb_theta->data.db[5] = sin(psi)*cos(theta)*cos(phi);
        R_eb_theta->data.db[6] = -cos(theta);
        R_eb_theta->data.db[7] = -sin(theta)*sin(phi);
        R_eb_theta->data.db[8] = -sin(theta)*cos(phi);
        
        R_eb_psi->data.db[0] = -sin(psi)*cos(theta);
        R_eb_psi->data.db[1] = -sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi);
        R_eb_psi->data.db[2] = -sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
        R_eb_psi->data.db[3] = cos(psi)*cos(theta);
        R_eb_psi->data.db[4] = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
        R_eb_psi->data.db[5] = cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
        R_eb_psi->data.db[6] = 0.0;
        R_eb_psi->data.db[7] = 0.0;
        R_eb_psi->data.db[8] = 0.0;
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
        for (int j = 0; j < cols; j++)
        {
            if(matrix->data.db[cols*i+j] >= 0.0)
                printf(" %.1f ",matrix->data.db[cols*i+j]);
            else
                printf("%.1f ",matrix->data.db[cols*i+j]);
        }
        cout << endl;
    }
    cout << endl;
}