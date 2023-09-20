#include <MIXEDVISION/CCalibrationPerspective.h>

#include <MIXEDVISION/CCalibrationPerspective.h>
#include <MIXEDVISION/CPosePerspective.h>

CCalibrationPerspective::CCalibrationPerspective(CPerspective *cam) : CCalibrationModel(cam)
{
	nbparamintr=4;
	if(cam != NULL)
		nbparamintr=cam->getNbActiveParameters();
}

CCalibrationPerspective::~CCalibrationPerspective()
{
}

void CCalibrationPerspective::setCamera(CModel *cam)
{
	init(cam);
	//nbparamintr = cam->getNbActiveParameters(); //4
}

void CCalibrationPerspective::computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix &Lsr)
{
    CPosePerspective pose;	
	pose.computeJacobianForVVS(P, cam, Lsr);
    /*
	CPosePerspective pose;
	vpMatrix Ls, Li(2,2);
	
	pose.computeJacobianForVVS(P, cam, Ls);
	if(cam->distorsions)
	{
		double x = P.get_x(), y = P.get_y(), r2, distR, xy, x2, y2;
        xy = x * y;
        x2 = x*x;
        y2 = y*y;
		r2 = x2+y2;
		distR = 1;
		if(cam->activek[0])
			distR += cam->k[0]*r2;
		if(cam->activek[1])
			distR += cam->k[1]*r2*r2; //r4
		if(cam->activek[2])
			distR += cam->k[2]*r2*r2*r2; //r6

		double facCom = cam->k[0]+2*cam->k[1]*r2+3*cam->k[2]*r2*r2;
		
		Li[0][0] = cam->getau() * (distR + 2*( x2*(facCom) + cam->k[3]*y + 3*cam->k[4]*x ));
		Li[0][1] = cam->getau() * (2*( xy*(facCom) + cam->k[3]*x + cam->k[4]*y ));
		
		Li[1][0] = cam->getav() * (2*( xy*(facCom) + cam->k[3]*x + cam->k[4]*y )); //(cam->getav()/cam->getau())*Li[0][1];
		Li[1][1] = cam->getav() * (distR + 2*( y2*(facCom) + 3*cam->k[3]*y + cam->k[4]*x ));
        
        //std::cout << Li << std::endl;
	}
	else
	{
		Li[0][0] = cam->getau(); 
		Li[1][1] = cam->getav();
	}
	
	Lsr = Li*Ls;*/
}

void CCalibrationPerspective::computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi)
{
	double x = P.get_x(), y = P.get_y();
	Lsi.resize(2,4);
    
	if(cam->distorsions)
	{
		//double au = cam->getau(), av = cam->getav();
		//Lsi.resize(2,cam->getNbActiveParameters());
		
		//double x = P.get_x(), y = P.get_y(), r2, distR, xy,
        double xd, yd;
        
       CPoint Ptmp = P;
        //cam->pixelMeterConversion(Ptmp); //attention : on veut obtenir les x,y distordus à partir des u, v distordus
        //xd = Ptmp.get_x();
        //yd = Ptmp.get_y();
        xd = (Ptmp.get_u() - cam->getu0())/cam->getau();
        yd = (Ptmp.get_v() - cam->getv0())/cam->getav();
        
        /*r2 = x*x+y*y;
		distR = 1;
		xy = x*y;
		if(cam->activek[0])
			distR += cam->k[0]*r2;
		if(cam->activek[1])
			distR += cam->k[1]*r2*r2; //r4
		if(cam->activek[2])
			distR += cam->k[2]*r2*r2*r2; //r6
		
		xd = x*distR;
		yd = y*distR;
		
		if(cam->activek[3])
		{
			xd += 2*cam->k[3]*xy;
			yd += cam->k[3]*(r2 + 2*y*y);
		}
		if(cam->activek[4])
		{
			xd += cam->k[4]*(r2 + 2*x*x);
			yd += 2*cam->k[4]*xy;
		}*/
		
		//Pour la premiere coordonnee
		Lsi[0][0] = xd;
		Lsi[0][1] = 0;
		Lsi[0][2] = 1.0;
		Lsi[0][3] = 0;
		
		//Pour la seconde coordonnee
		Lsi[1][0] = 0;
		Lsi[1][1] = yd;
		Lsi[1][2] = 0;
		Lsi[1][3] = 1.0;
		
        /*
		int ind = 4;
		if(cam->activek[0])
		{
			Lsi[0][ind] = au*x*r2;
			Lsi[1][ind] = av*y*r2;
			ind++;
		}
		if(cam->activek[1])
		{
			Lsi[0][ind] = au*x*r2*r2;
			Lsi[1][ind] = av*y*r2*r2;
			ind++;
		}
		if(cam->activek[2])
		{
			Lsi[0][ind] = au*x*r2*r2*r2;
			Lsi[1][ind] = av*y*r2*r2*r2;
			ind++;
		}
		if(cam->activek[3])
		{
			Lsi[0][ind] = 2*au*x*y;
			Lsi[1][ind] = av*(r2+2*y*y);
			ind++;
		}
		if(cam->activek[4])
		{
			Lsi[0][ind] = au*(r2+2*x*x);
			Lsi[1][ind] = 2*av*x*y;
			ind++;
		}     */   
	}
	else
	{
		//Lsi.resize(2,4);
		
		//Pour la premiere coordonnee
		Lsi[0][0] = x;
		Lsi[0][1] = 0;
		Lsi[0][2] = 1.0;
		Lsi[0][3] = 0;
		
		//Pour la seconde coordonnee
		Lsi[1][0] = 0;
		Lsi[1][1] = y;
		Lsi[1][2] = 0;
		Lsi[1][3] = 1.0;			
	}
}


void CCalibrationPerspective::updateCameraParameters(CModel *inCam, vpColVector & Tc_cam)
{
	CPerspective *cam = (CPerspective *)inCam;
	double k[5], ind = 4;
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
			  cam->getk1() + k[0], 
			  cam->getk2() + k[1], 
			  cam->getk3() + k[2], 
			  cam->getk4() + k[3], 
			  cam->getk5() + k[4]);
}

