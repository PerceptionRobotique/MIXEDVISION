#ifndef __COMMUN_H__
#define __COMMUN_H__

//#define GN
#define LM

#include <ostream>
#include <cstring>

typedef struct s_stats {
	int nbIter;
	double moy, std;
}calibStats, poseStats;

typedef enum{
	MIRE_DOTS,
	MIRE_CORNERS,
	MIRE_RINGS
} MireType;

typedef enum
{
	Omni,
	Persp,
	Fisheye,
	Paraboloid
} ModelType;

class ParamMire
{
public:
	double Lx, Ly, Lz;
	int nbx, nby, nbz;
	int nptPose;
	int pointsGrilleRef[4][3];
	
	ParamMire(double _Lx = 0.1, double _Ly = 0.1, double _Lz = 0.1, int _nbx = 6, int _nby = 6, int _nbz = 6, 
			  int **_pointsGrilleRef = NULL)//, int _nptPose = 4)
	{
		Lx = _Lx;
		Ly = _Ly;
		Lz = _Lz;
		
		nbx = _nbx;
		nby = _nby;
		nbz = _nbz;
		
		nptPose = 4;
		
		if(_pointsGrilleRef != NULL)
			for(int i = 0 ; i < nptPose ; i++)
				for(int j = 0 ; j < 3 ; j++)
					pointsGrilleRef[i][j] = _pointsGrilleRef[i][j];
	}

	~ParamMire() {}
	
	ParamMire& operator=(const ParamMire& pm)
	{
		Lx = pm.Lx;
		Ly = pm.Ly;
		Lz = pm.Lz;
		
		nbx = pm.nbx;
		nby = pm.nby;
		nbz = pm.nbz;
		
		nptPose = pm.nptPose;
		
		for(int i = 0 ; i < 4 ; i++)
			for(int j = 0 ; j < 3 ; j++)
				pointsGrilleRef[i][j] = pm.pointsGrilleRef[i][j];

		return *this;
	}
	

};

class MatricePose
{
public:
	MatricePose()
	{
		double *pdata = data;
		for(int i = 0 ; i < 4 ; i++)
		{
			rowPtrs[i] = pdata;
			for(int j = 0 ; j < 4 ; j++, pdata++)
			{	
				if(i == j)
					*pdata = 1.0;
				else
					*pdata = 0.0;
			}
		}
	}
	~MatricePose() {}
	
	//Pour écrire
	/*inline*/ double *operator[](int n){return rowPtrs[n];}
	//Pour lire
	/*inline*/ double *operator[](int n) const {return rowPtrs[n];}
	
	MatricePose &operator=(const MatricePose &M)
	{
		for(int i = 0 ; i < 4 ; i++)
			for(int j = 0 ; j < 4 ; j++)
				rowPtrs[i][j] = M[i][j];

		return *this;
	}

	/*!
	 \relates  vpHomogeneousMatrix
	 \brief invert the homogeneous matrix
	 
	 [R T]^-1 = [R^T  -R^T T]
	 
	 \return   [R T]^-1
	 */
	void inverse()
	{
		int i, j;
		double T[3] = {0.0, 0.0, 0.0};

		//bloc rotation transposé (Rt)
		for(i = 0 ; i < 3 ; i++)
			for(j = 0 ; j < 3 ; j++)
				rowPtrs[i][j] = rowPtrs[j][i];
		
		//T : -Rt * translation
		for (i = 0 ; i < 3; i++)
			for(j = 0 ; j < 3 ; j++)
				T[i] -= rowPtrs[i][j]*rowPtrs[j][3];
		
		//Affectation de T à la matrice courant
		for (i = 0 ; i < 3; i++)
			rowPtrs[i][3] = T[i];
	}
	
	MatricePose t()
	{
		MatricePose Mt ;
		int i,j;
		for (i=0;i<4;i++)
			for (j=0;j<4;j++)
				Mt[j][i] = (*this)[i][j];

		return Mt;
	}
	
private:
	double *rowPtrs[4];
public:
	double data[16];
};

class PoseVector
{
	public:
		PoseVector()
		{
			memset(data, 0, 6*sizeof(double));
		}

		//Pour écrire
		/*inline*/ double & operator[](int n){return data[n];}
		//Pour lire
		/*inline*/ double operator[](int n) const {return data[n];}

	PoseVector &operator=(const PoseVector &v)
	{
		for(int i = 0 ; i < 6 ; i++)
				data[i] = v[i];

		return *this;
	}

	private:
		double data[6];
};

typedef struct s_SImagePoint
{
    float u, v;
}SImagePoint;

#endif //__COMMUN_H__
