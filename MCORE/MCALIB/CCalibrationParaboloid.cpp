#include <MIXEDVISION/CCalibrationParaboloid.h>
#include <MIXEDVISION/CPoseParaboloid.h>

CCalibrationParaboloid::CCalibrationParaboloid(CParaboloid *cam) : CCalibrationModel(cam)
{
	nbparamintr = 4;//5;
}

CCalibrationParaboloid::~CCalibrationParaboloid()
{
}

void CCalibrationParaboloid::setCamera(CModel *cam)
{
	nbparamintr = 4;//5;
	init(cam);
}

void CCalibrationParaboloid::computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr)
{
    CPoseParaboloid pose;
	pose.computeJacobianForVVS(P, cam, Lsr);
    
	/*CPoseParaboloid pose;
	vpMatrix Ls, Li(2,2);

	pose.computeJacobianForVVS(P, cam, Ls);
	Li[0][0] = cam->getau(); 
	Li[1][1] = cam->getav();

	Lsr = Li*Ls;*/
}

void CCalibrationParaboloid::computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi)
{
	/*double au = cam->getau(), av = cam->getav(), h = ((CParaboloid *)cam)->geth();

	double x = P.get_x(), y = P.get_y();

	Lsi.resize(2,4);
    
    //Pour la premiere coordonnee
    Lsi[0][0]= x;
    Lsi[0][1]= 0;
    Lsi[0][2]= 1;
    Lsi[0][3]= 0;
    //Lsi[0][4]= au*x/h;
    
    //Pour la seconde coordonnee
    Lsi[1][0]= 0 ;
    Lsi[1][1]= y ;
    Lsi[1][2]= 0 ;
    Lsi[1][3]= 1;
    //Lsi[1][4]= av*y/h;
    */
    Lsi.resize(2,4);
    
	if(cam->distorsions)
	{
		double xd, yd;
        
        CPoint Ptmp = P;
        //cam->pixelMeterConversion(Ptmp); //attention : on veut obtenir les x,y distordus à partir des u, v distordus
        //xd = Ptmp.get_x();
        //yd = Ptmp.get_y();
        xd = (Ptmp.get_u() - cam->getu0())/cam->getau();
        yd = (Ptmp.get_v() - cam->getv0())/cam->getav();
        
        //Pour la premiere coordonnee
        Lsi[0][0]= xd;
        Lsi[0][1]= 0;
        Lsi[0][2]= 1;
        Lsi[0][3]= 0;

        //Pour la seconde coordonnee
        Lsi[1][0]= 0 ;
        Lsi[1][1]= yd ;
        Lsi[1][2]= 0 ;
        Lsi[1][3]= 1;
	}
    else
	{   
        double x = P.get_x(), y = P.get_y();
        
        //Pour la premiere coordonnee
        Lsi[0][0]= x;
        Lsi[0][1]= 0;
        Lsi[0][2]= 1;
        Lsi[0][3]= 0;
        //Lsi[0][4]= au*x/h;

        //Pour la seconde coordonnee
        Lsi[1][0]= 0 ;
        Lsi[1][1]= y ;
        Lsi[1][2]= 0 ;
        Lsi[1][3]= 1;
        //Lsi[1][4]= av*y/h;
    }
}

void CCalibrationParaboloid::updateCameraParameters(CModel *_cam, vpColVector & Tc_cam)
{
	CParaboloid *cam = (CParaboloid *)_cam;
    
	/*cam->init(cam->getau() + Tc_cam[0], cam->getav() + Tc_cam[1], 
		cam->getu0() + Tc_cam[2], cam->getv0() + Tc_cam[3], 
		cam->geth());// + Tc_cam[4]);
     */
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
              cam->geth(),
			  cam->getk1() + k[0], 
			  cam->getk2() + k[1], 
			  cam->getk3() + k[2], 
			  cam->getk4() + k[3], 
			  cam->getk5() + k[4]);

}
