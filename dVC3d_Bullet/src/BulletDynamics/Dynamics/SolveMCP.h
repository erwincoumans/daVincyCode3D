#ifndef _SOLVE_MCP_H_
#define _SOLVE_MCP_H_

extern "C" {
//#include <PATH/License.h>
}

#include <cassert>

#ifdef _MSC_VER 
 
	#ifdef _DEBUG
		#pragma comment (lib, "path46_debug.lib" )
	#else
		#pragma comment (lib, "path46.lib" )
	#endif

	#pragma comment(linker, "/nodefaultlib:dfor.lib /nodefaultlib:dfconsol.lib /nodefaultlib:dfport.lib")

#endif

extern "C"
{
#include "cs.h"
};

class csMatrix
{
public:
	csMatrix()
	{
		_mat = 0;
	}

	csMatrix(int m, int n, int nzmax, int values, int triplet)
	{
		_mat = cs_spalloc(m,n,nzmax,values,triplet);
	}

	~csMatrix()
	{
		if (_mat)
		{
			cs_spfree(_mat);
			_mat = 0;
		}
	}

	//double get (int i, int j = 0)
	//{
	//	return double(0);
	//}

	void set (int i,int j, double val)
	{
#ifdef _SPARSE_MATRIX_TOL 
		if (val > double(_SPARSE_MATRIX_TOL) || val < -double(_SPARSE_MATRIX_TOL) )
		{
			cs_entry(_mat, i, j, val);
			//printf("Set B[%d,%d]=%f\n",i,j,val);
		}
#else
		if (val != double(0) || val != -double(0))
		{
			cs_entry(_mat, i, j, val);
			//printf("Set B[%d,%d]=%f\n",i,j,val);
		}

#endif
	};

	int size1() const
	{
		if (_mat)
		{
			return _mat->m;
		}

		return -1;
	}

	int size2() const
	{
		if (_mat)
		{
			return _mat->n;
		}

		return -1;
	}

	void save(const char sName[]) const
	{
		FILE* f;
		f = fopen( sName, "w+" );

		int i = 0;
		
		assert(CS_TRIPLET(_mat));
		for (i=0; i < _mat->nz; i++)
		{
			fprintf(f, "%d %d %f\n",_mat->i[i]+1, _mat->p[i]+1, _mat->x[i]);
		}

		fclose(f);
	}

	void print(int brief)
	{
		cs_print(_mat,brief);
	}

	void dropTol(double tol)
	{
		cs_droptol(_mat,tol);
	}

	void dropZeroes()
	{
		cs_dropzeros(_mat);
	}

	cs*	_mat;
};

bool SolveMCP(double* outZ, double *lowerBound, double *upperBound,
			  const csMatrix & B,double* b);

#endif //_SOLVE_MCP_H_
