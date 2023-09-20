#include <MIXEDVISION/CCalibrationOmni.h>
#include <MIXEDVISION/CPoseOmni.h>

CCalibrationOmni::CCalibrationOmni(COmni *cam) : CCalibrationModel(cam)
{
	nbparamintr = 5;//ATTENTION : juste pour fixer xi a 1 //4; //
	if(cam != NULL)
		nbparamintr=cam->getNbActiveParameters();
}

CCalibrationOmni::~CCalibrationOmni()
{
}

void CCalibrationOmni::setCamera(CModel *cam)
{
	init(cam);
	//nbparamintr = 5;
}

void CCalibrationOmni::computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr)
{
	CPoseOmni pose;
	pose.computeJacobianForVVS(P, cam, Lsr);
	/*vpMatrix Ls, Li(2,2);
	pose.computeJacobianForVVS(P, cam, Ls);
	Li[0][0] = cam->getau(); 
	Li[1][1] = cam->getav();

	Lsr = Li*Ls;*/
}

void CCalibrationOmni::computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi)
{
	/*double au = cam->getau(), av = cam->getav(), xi = ((COmni *)cam)->getXi();

	double x = P.get_x(), y = P.get_y();
	double X = P.get_X(), Y = P.get_Y(), Z = P.get_Z();
	double rho = sqrt(X*X+Y*Y+Z*Z);

	Lsi.resize(2,5);

	//Pour la premiere coordonnee
	Lsi[0][0]= x;
	Lsi[0][1]= 0;
	Lsi[0][2]= 1;
	Lsi[0][3]= 0;
	Lsi[0][4]= -au*rho*x/(Z + xi*rho);
	//Pour la seconde coordonnee
	Lsi[1][0]= 0 ;
	Lsi[1][1]= y ;
	Lsi[1][2]= 0 ;
	Lsi[1][3]= 1;
	Lsi[1][4]= -av*rho*y/(Z + xi*rho);*/
    double au = cam->getau(), av = cam->getav(), xi = ((COmni *)cam)->getXi();
    
    double x = P.get_x(), y = P.get_y();
    double X = P.get_X(), Y = P.get_Y(), Z = P.get_Z();
    double rho = sqrt(X*X+Y*Y+Z*Z);
    
    Lsi.resize(2,5);
    
	if(cam->distorsions)
	{
		double r2, distR, xy, xd, yd, x2, y2;
        
        CPoint Ptmp = P;
        // cam->pixelMeterConversion(Ptmp); //attention : on veut obtenir les x,y distordus à partir des u, v distordus
        // xd = Ptmp.get_x();
        // yd = Ptmp.get_y();
        xd = (Ptmp.get_u() - cam->getu0())/cam->getau();
        yd = (Ptmp.get_v() - cam->getv0())/cam->getav();
        
        x2 = x*x; y2 = y*y;
        r2 = x2+y2;
		distR = 1;
		if(cam->activek[0])
			distR += cam->k[0]*r2;
		if(cam->activek[1])
			distR += cam->k[1]*r2*r2; //r4
		if(cam->activek[2])
			distR += cam->k[2]*r2*r2*r2; //r6
		
		/*
		xy = x*y;
        xd = x*distR;
		yd = y*distR;

		if(cam->activek[3])
		{
			xd += 2*cam->k[3]*xy;
			yd += cam->k[3]*(r2 + 2*y2);
		}
		if(cam->activek[4])
		{
			xd += cam->k[4]*(r2 + 2*x2);
			yd += 2*cam->k[4]*xy;
		}*/
        
        double factCom = cam->k[0]*r2 + 2*cam->k[1]*r2*r2 + 3*cam->k[2]*r2*r2*r2 + 0.5*distR;
        
        //Pour la premiere coordonnee
        Lsi[0][0]= xd;
        Lsi[0][1]= 0;
        Lsi[0][2]= 1;
        Lsi[0][3]= 0;
        //Lsi[0][4]= -(au*x*rho/(Z+xi*rho))*(x*(2*x*cam->k[0]+4*x*cam->k[1]*r2+6*x*cam->k[2]*r2*r2)+cam->k[0]*r2+cam->k[1]*r2*r2+cam->k[2]*r2*r2*r2+1+2*cam->k[3]*y+6*x*cam->k[4])-(au*y*rho/(Z+xi*rho))*(x*(2*y*cam->k[0]+4*y*cam->k[1]*r2+6*y*cam->k[2]*r2*r2)+2*x*cam->k[3]+2*y*cam->k[4]);
        Lsi[0][4]= -2*(au*rho*x/(Z + xi*rho))*(cam->k[4]*(3*x2+y2)/x + factCom + 2*y*cam->k[3]);
        //Lsi[0][4]= -2*(rho/(Z + xi*rho))*(P.get_u()-cam->getu0() + au*x*(cam->k[0]*r2 + 2*cam->k[1]*r2*r2 + 3*cam->k[2]*r2*r2*r2) - au*0.5*x*distR);
        //Pour la seconde coordonnee
        Lsi[1][0]= 0 ;
        Lsi[1][1]= yd ;
        Lsi[1][2]= 0 ;
        Lsi[1][3]= 1;
        //Lsi[1][4]= -(av*y*rho/(Z+xi*rho))*(y*(2*y*cam->k[0]+4*y*cam->k[1]*r2+6*y*cam->k[2]*r2*r2)+cam->k[0]*r2+cam->k[1]*r2*r2+cam->k[2]*r2*r2*r2+1+6*y*cam->k[3]*y+2*x*cam->k[4])-(av*x*rho/(Z+xi*rho))*(y*(2*x*cam->k[0]+4*x*cam->k[1]*r2+6*x*cam->k[2]*r2*r2)+2*x*cam->k[3]+2*y*cam->k[4]);
            Lsi[1][4]= -2*(av*rho*y/(Z + xi*rho))*(cam->k[3]*(x2+3*y2)/y + factCom + 2*x*cam->k[4]);
        //Lsi[1][4]= -2*(rho/(Z + xi*rho))*(P.get_v()-cam->getv0() + av*y*(cam->k[0]*r2 + 2*cam->k[1]*r2*r2 + 3*cam->k[2]*r2*r2*r2) - av*0.5*y*distR);
	}
    else
	{
        //Pour la premiere coordonnee
        Lsi[0][0]= x;
        Lsi[0][1]= 0;
        Lsi[0][2]= 1;
        Lsi[0][3]= 0;
        Lsi[0][4]= -au*rho*x/(Z + xi*rho);
        //Pour la seconde coordonnee
        Lsi[1][0]= 0 ;
        Lsi[1][1]= y ;
        Lsi[1][2]= 0 ;
        Lsi[1][3]= 1;
        Lsi[1][4]= -av*rho*y/(Z + xi*rho);
	}
    //Lsi.resize(2,4,false); //ATTENTION : juste pour fixer xi a 1
}

void CCalibrationOmni::updateCameraParameters(CModel *_cam, vpColVector & Tc_cam)
{
	COmni *cam = (COmni *)_cam;

/*	cam->init(cam->getau() + Tc_cam[0], cam->getav() + Tc_cam[1], 
		cam->getu0() + Tc_cam[2], cam->getv0() + Tc_cam[3], 
		cam->getXi() + Tc_cam[4]);*/
    double k[5], ind = 5;//ATTENTION : juste pour fixer xi a 1; //4;//
	for(int i = 0 ; i < 5 ; i++)
	{
		if(cam->activek[i])
		{
			k[i] = Tc_cam[ind];
			ind++;
		}
		else
			k[i] = 0.0;
	}
	
	cam->init(cam->getau() + Tc_cam[0], 
			  cam->getav() + Tc_cam[1],
			  cam->getu0() + Tc_cam[2], 
			  cam->getv0() + Tc_cam[3], 
              cam->getXi() + Tc_cam[4], //ATTENTION : juste pour fixer xi a 1 //, //
			  cam->getk1() + k[0], 
			  cam->getk2() + k[1], 
			  cam->getk3() + k[2], 
			  cam->getk4() + k[3], 
			  cam->getk5() + k[4]);
}

