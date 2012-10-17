/*
   Bullet Continuous Collision Detection and Physics Library
   Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

   This software is provided 'as-is', without any express or implied warranty.
   In no event will the authors be held liable for any damages arising from the use of this software.
   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it freely,
   subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
   2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
   3. This notice may not be removed or altered from any source distribution.
 */

/*
        dvc3D library: an accurate physics engine.
        Author : Binh Nguyen ( www.cs.rpi.edu/~nguyeb2 )
        This file has the same license as Bullet.

        ST time stepper (short for Stewart-Trinkle, two main authors of the first paper described the method).

 */

#include "btSTModWorld.h"

#include "BulletDynamics/Dynamics/SolveMCP.h"

#define USE_ST_METHOD 1
//#define _SAVE_LCP_ 1

const char PATH_LICENSE[] = "2858585124&Jeff_Trinkle&Rensselaer_Polytechnic_Institute&&USR&45879&10_1_2011&1000&PATH&GEN&0_0_0&0_0_0&5000&0_0";

btSTModWorld::btSTModWorld( btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration )
	: btDiscreteDynamicsWorld( dispatcher,pairCache,constraintSolver,collisionConfiguration )
	,m_constraintSolver( constraintSolver )
	,m_gravity( 0,-10,0 )
	,m_localTime( 0.0f )
	,m_synchronizeAllMotionStates( true )
	,m_profileTimings( 0 )
	,m_frictionDirs( 3 )
	,m_stablizationFraction( 0.9f )
	,m_useSpeculativeContact( true )
	,m_problemSize(0)
	,m_LCPsolutionTime(0)
#ifdef _SAVE_LCP_
	,m_LCPFrame(0)
#endif
{
	if ( !m_constraintSolver )
	{
		void* mem = btAlignedAlloc( sizeof( btSequentialImpulseConstraintSolver ),16 );
		m_constraintSolver     = new (mem)btSequentialImpulseConstraintSolver;
		m_ownsConstraintSolver = true;
	}
	else
		m_ownsConstraintSolver = false;

	{
		void* mem = btAlignedAlloc( sizeof( btSimulationIslandManager ),16 );
		m_islandManager = new (mem)btSimulationIslandManager();
	}

	m_ownsIslandManager = true;

#if USE_ST_METHOD
	License_SetString( PATH_LICENSE );

	if ( m_frictionDirs )
		generateLocalFrictions();

#endif // if USE_ST_METHOD
}

btSTModWorld::~btSTModWorld(void)
{
	// only delete it when we created it
	if ( m_ownsIslandManager )
	{
		m_islandManager->~btSimulationIslandManager();
		btAlignedFree( m_islandManager );
	}

	if ( m_ownsConstraintSolver )
	{
		m_constraintSolver->~btConstraintSolver();
		btAlignedFree( m_constraintSolver );
	}
}