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

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "LinearMath/btQuickprof.h"
#include "json/elements.h"
#include "json/writer.h"

class btSTModWorld : public btDiscreteDynamicsWorld
{
public:
	// /this btSTWorld constructor gets created objects from the user, and will not delete those
	btSTModWorld( btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration );

	virtual ~btSTModWorld();

	/***********************************************************************
		Additional stuffs for ST time-stepper
	***********************************************************************/
	int getFrictionDirections() const
	{
		return m_frictionDirs;
	} // getFrictionDirections

	void setFrictionDirections( int nfrictionDirs )
	{
		if ( nfrictionDirs != m_frictionDirs )
		{
			m_frictionDirs = nfrictionDirs;
			generateLocalFrictions();
		}
	} // setFrictionDirections

	btScalar getStablizationFraction() const
	{
		return m_stablizationFraction;
	} // getStablizationFraction

	void setStablizationFraction( const btScalar & f )
	{
		m_stablizationFraction = f;
	} // setStablizationFraction

	bool getUseSpeculativeContact() const
	{
		return m_useSpeculativeContact;
	} // getUseSpeculativeContact

	void setUseSpeculativeContact( bool bUse )
	{
		m_useSpeculativeContact = bUse;
	} // setUseSpeculativeContact

	unsigned int m_LCPFrame;

	// Profile data
	btClock		m_clock;

	long		m_LCPsolutionTime; // in microsec
	int			m_problemSize;

	json::Object m_timeStepInfo;

	std::string  m_jsonString;

	const std::string getJsonInfo() const
	{
		return m_jsonString;
	}

	/***********************************************************************
		End
	***********************************************************************/

protected:
	/***********************************************************************
		Additional stuffs for ST time-stepper
	***********************************************************************/
	// Friction related
	int m_frictionDirs;

	btAlignedObjectArray < btVector3 > m_localFrictionDirs;

	void generateLocalFrictions ();

	btScalar m_stablizationFraction;

	bool m_useSpeculativeContact;
	/***********************************************************************
		End
	***********************************************************************/
};