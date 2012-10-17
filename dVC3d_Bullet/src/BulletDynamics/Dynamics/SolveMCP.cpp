#include <iostream>
#include "BulletDynamics\Dynamics\Standalone_Path.h"
extern "C"
{
	#include "BulletDynamics\Dynamics\SimpleLCP.h"
};

//#include "Matrix.h"
//#include "DiagMatrix.h"
#include "SolveMCP.h"

static const char* gReturnMsgs[] = {
	"Invalid return",
	"MCP_Solved: The problem was solved",
	"MCP_NoProgress: A stationary point was found",
	"MCP_MajorIterationLimit: Major iteration limit met",
	"MCP_MinorIterationLimit: Cumulative minor iteration limit met",
	"MCP_TimeLimit: Ran out of time",
	"MCP_UserInterrupt: Control-C, typically",
	"MCP_BoundError: Problem has a bound error",
	"MCP_DomainError: Could not find starting point",
	"MCP_Infeasible: Problem has no solution",
	"MCP_Error: An error occurred within the code",
	"MCP_LicenseError: License could not be found",
	"MCP_OK"
};

/**
* Wrapper to call Path's simple LCP solver
*/
bool SolveMCP(double* outZ, double *lowerBound, double *upperBound,
			  const csMatrix & B, double* b)
{
	assert(B.size1() == B.size2());
	bool result;

	unsigned int i = 0;

	size_t dimension = B.size1();
	if (dimension == 0)
	{ // Nothing to do
		return true;
	}
	//cs_print(B._mat,0);
	int numNonZero = B._mat->nz ;

	assert(numNonZero >= 0);

	int* rowArray = new int[numNonZero];
	int* colArray = new int[numNonZero];
	double* valArray = B._mat->x;

	for (i=0; i< numNonZero; i++)
	{
		rowArray[i] = B._mat->i[i] +1;
		colArray[i] = B._mat->p[i] +1;
	}

	double* bArray = b;

	MCP_Termination status;
	// Call Path's solver
	SimpleLCP(dimension,
		numNonZero,
		rowArray,
		colArray,
		valArray,
		bArray,
		lowerBound,
		upperBound,
		& status,
		outZ);

	if (status != MCP_Solved)
	{
		std::cerr << "ERROR: The PATH MCP solver failed: "  << gReturnMsgs[(unsigned int)status] << std::endl;
		result = false;
	}
	else
		result = true;

	// Cleanup
	delete[] rowArray;
	delete[] colArray;
	//delete[] valArray;
	//delete[] bArray;

	// Return types can be found in path_include/Types.h
	return result;
}