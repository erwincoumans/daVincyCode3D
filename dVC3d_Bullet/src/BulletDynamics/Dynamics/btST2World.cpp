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

#include "btST2World.h"
extern "C"
{
#include "Path/License.h"
};

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btQuickprof.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMotionState.h"

#include "LinearMath/btSerializer.h"

/***********************************************************************
   Additional stuffs for ST time-stepper
***********************************************************************/
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "SolveMCP.h"

#define USE_ST_METHOD 1
//#define _SAVE_LCP_ 1


/*
  **********************************************************************
   End
  ***********************************************************************/

btST2World::btST2World(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration)
	: btDynamicsWorld( dispatcher,pairCache,collisionConfiguration )
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
	if (!m_constraintSolver)
	{
		void* mem = btAlignedAlloc(sizeof(btSequentialImpulseConstraintSolver),16);
		m_constraintSolver = new (mem) btSequentialImpulseConstraintSolver;
		m_ownsConstraintSolver = true;
	} else
	{
		m_ownsConstraintSolver = false;
	}

	{
		void* mem = btAlignedAlloc(sizeof(btSimulationIslandManager),16);
		m_islandManager = new (mem) btSimulationIslandManager();
	}

	m_ownsIslandManager = true;

	#if USE_ST_METHOD
	License_SetString("2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0");
	
	//License_SetString("2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0");
//	License_SetString( PATH_LICENSE );

	if ( m_frictionDirs )
		generateLocalFrictions();

	#endif // if USE_ST_METHOD
}

btST2World::~btST2World()
{
	//only delete it when we created it
	if (m_ownsIslandManager)
	{
		m_islandManager->~btSimulationIslandManager();
		btAlignedFree( m_islandManager);
	}
	if (m_ownsConstraintSolver)
	{
		m_constraintSolver->~btConstraintSolver();
		btAlignedFree(m_constraintSolver);
	}
}

void	btST2World::saveKinematicState(btScalar timeStep)
{
	///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
	///to switch status _after_ adding kinematic objects to the world
	///fix it for Bullet 3.x release
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body && body->getActivationState() != ISLAND_SLEEPING)
		{
			if (body->isKinematicObject())
			{
				//to calculate velocities next frame
				body->saveKinematicState(timeStep);
			}
		}
	}
}

void	btST2World::debugDrawWorld()
{
	BT_PROFILE("debugDrawWorld");

	btCollisionWorld::debugDrawWorld();

	bool drawConstraints = false;
	if (getDebugDrawer())
	{
		int mode = getDebugDrawer()->getDebugMode();
		if(mode  & (btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if(drawConstraints)
	{
		for(int i = getNumConstraints()-1; i>=0 ;i--)
		{
			btTypedConstraint* constraint = getConstraint(i);
			debugDrawConstraint(constraint);
		}
	}

	if (getDebugDrawer() && getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb))
	{
		int i;

		if (getDebugDrawer() && getDebugDrawer()->getDebugMode())
		{
			for (i=0;i<m_actions.size();i++)
			{
				m_actions[i]->debugDraw(m_debugDrawer);
			}
		}
	}
}

void	btST2World::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		//need to check if next line is ok
		//it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
		body->clearForces();
	}
}

///apply gravity, call this once per timestep
void	btST2World::applyGravity()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive())
		{
			body->applyGravity();
		}
	}
}

void	btST2World::synchronizeSingleMotionState(btRigidBody* body)
{
	btAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we need to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		///@todo: add 'dirty' flag
		//if (body->getActivationState() != ISLAND_SLEEPING)
		{
			btTransform interpolatedTransform;
			btTransformUtil::integrateTransform(body->getInterpolationWorldTransform(),
				body->getInterpolationLinearVelocity(),body->getInterpolationAngularVelocity(),m_localTime*body->getHitFraction(),interpolatedTransform);
			body->getMotionState()->setWorldTransform(interpolatedTransform);
		}
	}
}

void	btST2World::synchronizeMotionStates()
{
	BT_PROFILE("synchronizeMotionStates");
	if (m_synchronizeAllMotionStates)
	{
		//iterate  over all collision objects
		for ( int i=0;i<m_collisionObjects.size();i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
				synchronizeSingleMotionState(body);
		}
	} else
	{
		//iterate over all active rigid bodies
		for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
		{
			btRigidBody* body = m_nonStaticRigidBodies[i];
			if (body->isActive())
				synchronizeSingleMotionState(body);
		}
	}
}

int	btST2World::stepSimulation( btScalar timeStep,int maxSubSteps, btScalar fixedTimeStep)
{
	startProfiling(timeStep);

	BT_PROFILE("stepSimulation");

	int numSimulationSubSteps = 0;

	if (maxSubSteps)
	{
		//fixed timestep with interpolation
		m_localTime += timeStep;
		if (m_localTime >= fixedTimeStep)
		{
			numSimulationSubSteps = int( m_localTime / fixedTimeStep);
			m_localTime -= numSimulationSubSteps * fixedTimeStep;
		}
	} else
	{
		//variable timestep
		fixedTimeStep = timeStep;
		m_localTime = timeStep;
		if (btFuzzyZero(timeStep))
		{
			numSimulationSubSteps = 0;
			maxSubSteps = 0;
		} else
		{
			numSimulationSubSteps = 1;
			maxSubSteps = 1;
		}
	}

	//process some debugging flags
	if (getDebugDrawer())
	{
		btIDebugDraw* debugDrawer = getDebugDrawer ();
		gDisableDeactivation = (debugDrawer->getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0;
	}
	if (numSimulationSubSteps)
	{
		//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
		int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps)? maxSubSteps : numSimulationSubSteps;

		saveKinematicState(fixedTimeStep*clampedSimulationSteps);

		applyGravity();

		for (int i=0;i<clampedSimulationSteps;i++)
		{
			internalSingleStepSimulation(fixedTimeStep);
			synchronizeMotionStates();
		}
	} else
	{
		synchronizeMotionStates();
	}

	clearForces();

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE

	return numSimulationSubSteps;
}

void	btST2World::internalSingleStepSimulation(btScalar timeStep)
{
	BT_PROFILE( "internalSingleStepSimulation" );

	if ( 0 != m_internalPreTickCallback )
		( *m_internalPreTickCallback )( this, timeStep );

	// /apply gravity, predict motion
	predictUnconstraintMotion( timeStep );

	btDispatcherInfo & dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep  = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	// Use predicted transformations
	if ( m_useSpeculativeContact )
	{
		btAlignedObjectArray < btTransform > saveTransforms;
		saveTransforms.resize( m_nonStaticRigidBodies.size() );

		for ( int i = 0; i < m_nonStaticRigidBodies.size(); i++ )
		{
			btRigidBody* body = m_nonStaticRigidBodies[i];

			if ( !body->isStaticObject() )
			{
				// save current transform
				saveTransforms[i] = body->getWorldTransform();
				body->setWorldTransform( body->getInterpolationWorldTransform() );
				body->predictIntegratedTransform( timeStep, body->getInterpolationWorldTransform() );
			}
		}

		// /perform collision detection
		performDiscreteCollisionDetection();

		for ( int i = 0; i < m_nonStaticRigidBodies.size(); i++ )
		{
			btRigidBody* body = m_nonStaticRigidBodies[i];

			if ( !body->isStaticObject() )
			{
				// save current transform
				body->setInterpolationWorldTransform( body->getWorldTransform() );
				body->setWorldTransform( saveTransforms[i] );
			}
		}
	}
	else
		// /perform collision detection
		performDiscreteCollisionDetection();

	#if USE_ST_METHOD

	/*
	  **********************************************************************
	                      Stewart-Trinkle time-stepper
	  ***********************************************************************/
	int numTotalBodies = getNumCollisionObjects();

	// printf("Total bodies: %d\n", numTotalBodies);

	int numManifolds = getDispatcher()->getNumManifolds();
	int numContacts  = 0;
	int i = 0;

	// get number of contacts
	for ( i = 0; i < numManifolds; i++ )
	{
		btPersistentManifold* contactManifold = getDispatcher()->getManifoldByIndexInternal( i );
		btCollisionObject   * obA = static_cast < btCollisionObject * > ( contactManifold->getBody0() );
		btCollisionObject   * obB = static_cast < btCollisionObject * > ( contactManifold->getBody1() );

		// contactManifold->setContactBreakingThreshold(0.1);
		// contactManifold->refreshContactPoints(obA->getWorldTransform(),obB->getWorldTransform());
		const btTransform & trA = obA->getWorldTransform();
		const btTransform & trB = obB->getWorldTransform();

		// TODO : change m_normalWorldOnB
		if (m_useSpeculativeContact)
		{
			for ( int iContact = contactManifold->getNumContacts() - 1; iContact >= 0; iContact-- )
			{
				btManifoldPoint & manifoldPoint = contactManifold->getContactPoint( iContact );

				manifoldPoint.m_positionWorldOnA = trA( manifoldPoint.m_localPointA );
				manifoldPoint.m_positionWorldOnB = trB( manifoldPoint.m_localPointB );
				manifoldPoint.m_distance1 = ( manifoldPoint.m_positionWorldOnA - manifoldPoint.m_positionWorldOnB ).dot( manifoldPoint.m_normalWorldOnB );
				manifoldPoint.m_lifeTime++;
			}
		}
		else
		{
			contactManifold->refreshContactPoints(trA,trB);
		}

		numContacts += contactManifold->getNumContacts();
	}

	// get number of constraints (joints)
	int numJoints = getNumConstraints();

	if ( numContacts + numJoints )
	{
		// get number of movable bodies
		int                    numMovableBodies = 0;
		btCollisionObjectArray & bodyList = getCollisionObjectArray();

		btAlignedObjectArray < int > bodyIndexArray; // map movable body index to our bodies list

		for ( i = 0; i < numTotalBodies; i++ )
		{
			btRigidBody* pBody = btRigidBody::upcast( bodyList[i] );

			if ( pBody )
			{
				if ( !bodyList[i]->isStaticOrKinematicObject() )
				{
					pBody->m_bodyIdx = numMovableBodies;

					bodyIndexArray.push_back( i );
					numMovableBodies++;

					if ( pBody->isActive() )
					{
						// update external forces
						// pBody->applyCentralForce( pBody->getGravity() );
						// body->integrateVelocities( timeStep);
						// body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
					}
				}
				else
					// not movable body should have index -1
					pBody->m_bodyIdx =  -1 ;
			}
		}

		// printf( "%d.\n" , numMovableBodies );

		/***********************************************************************
		   Get size of Wb
		 ***********************************************************************/
		btTypedConstraint::btConstraintInfo1 info1;
		int                                  numConstraints = 0;

		for ( i = 0; i < numJoints; i++ )
		{
			btTypedConstraint* pConstraint = getConstraint( i );

			if ( CONTACT_CONSTRAINT_TYPE == pConstraint->getConstraintType() )
				continue;

			pConstraint->getInfo1( &info1 );
			// pConstraint->getInfo2(&info2);
			// debugging
			// printf("Info1: m_numConstraintRows = %d, nub = %d.\n", info1.m_numConstraintRows, info1.nub);

			numConstraints += info1.m_numConstraintRows;
			pConstraint->buildJacobian();
		}

		// printf("NumConstraints = %d.\n",numConstraints);

		// Matrix size
		int n = 0;

		if ( m_frictionDirs )
			// M                 Wb                Wn                   Wf                  E
			n = 6 * numMovableBodies + numConstraints + numContacts + numContacts * m_frictionDirs + numContacts;
		else
			n = 6 * numMovableBodies + numConstraints + numContacts;

		// @@@@@@@@@@
		{
			m_problemSize = n;
			m_timeStepInfo["ProblemSize"] = json::Number(n);
			m_timeStepInfo["FrictionDirs"] = json::Number( m_frictionDirs );
			m_timeStepInfo["NumContacts"] = json::Number( numContacts );
			m_timeStepInfo["NumConstraints"] = json::Number( numConstraints);
			m_timeStepInfo["NumBodies"] = json::Number( numMovableBodies );
		}

		double* l = new double[n];  /* Lower bounds on the variables       */
		double* u = new double[n];  /* Upper bounds on the variables       */

		// DEBUG
		// printf("LCP size : %d\n", n );

		// Matrix<double> B(n,n);
		csMatrix B( 0,0,1,1,1 );

		btAlignedObjectArray < double > b;
		b.resize( n );

		// Fill M part
		for ( i = 0; i < numMovableBodies; i++ )
		{
			btRigidBody* pBody = btRigidBody::upcast( bodyList[bodyIndexArray[i]] );
			// diagonal part
			btScalar mass = 1.0f / pBody->getInvMass();
			// inertia part
			// btVector3 localInertia = pBody->getInvInertiaDiagLocal();
			const btMatrix3x3 & worldInertia = pBody->getInvInertiaTensorWorld().inverse();

			const btVector3 & linVel     = pBody->getLinearVelocity();
			const btVector3 & angularVel = pBody->getAngularVelocity();
			const btVector3 & extForce   = pBody->getTotalForce();
			const btVector3 & extTorque  = pBody->getTotalTorque();

			B.set( 6 * i,6 * i, -mass );
			B.set( 6 * i + 1,6 * i + 1, -mass );
			B.set( 6 * i + 2,6 * i + 2, -mass );

			// B.set(6*i+3,6*i+3, -1.0f / localInertia.x());
			// B.set(6*i+4,6*i+4, -1.0f / localInertia.y());
			// B.set(6*i+5,6*i+5, -1.0f / localInertia.z());

			B.set( 6 * i + 3,6 * i + 3,-worldInertia[0].x() );
			B.set( 6 * i + 3,6 * i + 4,-worldInertia[0].y() );
			B.set( 6 * i + 3,6 * i + 5,-worldInertia[0].z() );

			B.set( 6 * i + 4,6 * i + 3,-worldInertia[1].x() );
			B.set( 6 * i + 4,6 * i + 4,-worldInertia[1].y() );
			B.set( 6 * i + 4,6 * i + 5,-worldInertia[1].z() );

			B.set( 6 * i + 5,6 * i + 3,-worldInertia[2].x() );
			B.set( 6 * i + 5,6 * i + 4,-worldInertia[2].y() );
			B.set( 6 * i + 5,6 * i + 5,-worldInertia[2].z() );

			b[ 6 * i + 0 ] = mass * linVel.x() + extForce.x() * timeStep;
			b[ 6 * i + 1 ] = mass * linVel.y() + extForce.y() * timeStep;
			b[ 6 * i + 2 ] = mass * linVel.z() + extForce.z() * timeStep;
			// b(6*i + 3)      = 1.0f/localInertia.x()*angularVel.x() + extTorque.x()*timeStep;
			// b(6*i + 4)      = 1.0f/localInertia.y()*angularVel.y() + extTorque.y()*timeStep;
			// b(6*i + 5)      = 1.0f/localInertia.z()*angularVel.z() + extTorque.z()*timeStep;

			const btVector3 & coriolis = worldInertia * angularVel;
			b[ 6 * i + 3 ] = coriolis.x() + extTorque.x() * timeStep;
			b[ 6 * i + 4 ] = coriolis.y() + extTorque.y() * timeStep;
			b[ 6 * i + 5 ] = coriolis.z() + extTorque.z() * timeStep;

			// printf("Extforce = [%f,%f,%f]\n",extForce.x(), extForce.y(), extForce.z());
		}

		// Wb part
		int iConstraint = 0;
		int iStartColWb = 6 * numMovableBodies;

		if ( numConstraints )
		{
			int                                  iJoint = 0;
			int                                  iLocalConstraint = 0;
			btTypedConstraint::btConstraintInfo2 info2;
			int                                  iCIdx    = 0;
			static const int		     nRowSkip = 3;

			// Jacobian storage
			btScalar jl0[nRowSkip * 6];
			btScalar ja0[nRowSkip * 6];
			btScalar jl1[nRowSkip * 6];
			btScalar ja1[nRowSkip * 6];
			btScalar jErr[nRowSkip * 6];
			btScalar lLimits[nRowSkip * 6];
			btScalar uLimits[nRowSkip * 6];
			btScalar cfm[nRowSkip * 6];

			// initialize unchanged info2
			info2.cfm     = &cfm[0];
			info2.fps     = btScalar( 1.0 ) / timeStep;
			info2.erp     = btScalar( 0.5 );
			info2.rowskip = nRowSkip;

			info2.m_J1linearAxis  = &jl0[0];
			info2.m_J1angularAxis = &ja0[0];
			info2.m_J2linearAxis  = &jl1[0];
			info2.m_J2angularAxis = &ja1[0];

			info2.m_constraintError = &jErr[0];
			info2.m_lowerLimit = &lLimits[0];
			info2.m_upperLimit = &uLimits[0];

			for ( iJoint = 0; iJoint < numJoints; iJoint++ )
			{
				btTypedConstraint* pConstraint = getConstraint( iJoint );

				if ( CONTACT_CONSTRAINT_TYPE == pConstraint->getConstraintType() )
				{
					// Should not happen but to be safe.
					printf( " [WARNING]: Bullet contact joint appears in dvc3D.\n" );
					continue;
				}

				// Get constraints size
				pConstraint->getInfo1( &info1 );

				int index0 = pConstraint->getRigidBodyA().m_bodyIdx;
				int index1 = pConstraint->getRigidBodyB().m_bodyIdx;

				// printf("Bodyidx = [%d %d].\n", index0,index1);

				// clear data
				for ( int ii = 0; ii < info1.m_numConstraintRows; ii++ )
				{
					jl0[nRowSkip * ii]     = 0.0;
					jl0[nRowSkip * ii + 1] = 0.0;
					jl0[nRowSkip * ii + 2] = 0.0;

					ja0[nRowSkip * ii]     = 0.0;
					ja0[nRowSkip * ii + 1] = 0.0;
					ja0[nRowSkip * ii + 2] = 0.0;

					jl1[nRowSkip * ii]     = 0.0;
					jl1[nRowSkip * ii + 1] = 0.0;
					jl1[nRowSkip * ii + 2] = 0.0;

					ja1[nRowSkip * ii]     = 0.0;
					ja1[nRowSkip * ii + 1] = 0.0;
					ja1[nRowSkip * ii + 2] = 0.0;

					lLimits[nRowSkip * ii]	   = -btScalar( 1e20 );
					lLimits[nRowSkip * ii + 1] = -btScalar( 1e20 );
					lLimits[nRowSkip * ii + 2] = -btScalar( 1e20 );

					uLimits[nRowSkip * ii]	   = btScalar( 1e20 );
					uLimits[nRowSkip * ii + 1] = btScalar( 1e20 );
					uLimits[nRowSkip * ii + 2] = btScalar( 1e20 );

					// jErr[ii] = 0.0;
				}

				pConstraint->getInfo2( &info2 );
				// debugging
				// btTypedConstraint::printInfo(info1,info2);
				// printf("Info1: m_numConstraintRows = %d, nub = %d.\n", info1.m_numConstraintRows, info1.nub);
				// printf("\n");
				// for (int ii = 0; ii < info1.m_numConstraintRows; ii++)
				// {
				// printf("%f ",jErr[ii * nRowSkip]);
				// //printf("%f %f %f ",ja0[3*ii],ja0[3*ii+1],ja0[3*ii+2]);
				// //ja0[3*ii] = ja0[3*ii+1] = ja0[3*ii+2] = 0.0;
				// //jl1[3*ii] = jl1[3*ii+1] = jl1[3*ii+2] = 0.0;
				// //ja1[3*ii] = ja1[3*ii+1] = ja1[3*ii+2] = 0.0;

				// //jErr[ii] = 0.0;
				// }
				// printf(".\n");

				if ( index0 != -1 )
					for ( iLocalConstraint = 0; iLocalConstraint < info1.m_numConstraintRows; iLocalConstraint++ )
					{
						// column index of iLocalConstraint
						iCIdx = iStartColWb + iConstraint + iLocalConstraint;
						// linear part
						B.set( iCIdx, 6 * index0 + 0, jl0[nRowSkip * iLocalConstraint + 0] );
						B.set( iCIdx, 6 * index0 + 1, jl0[nRowSkip * iLocalConstraint + 1] );
						B.set( iCIdx, 6 * index0 + 2, jl0[nRowSkip * iLocalConstraint + 2] );
						// transpose
						B.set( 6 * index0 + 0,iCIdx,  jl0[nRowSkip * iLocalConstraint + 0] );
						B.set( 6 * index0 + 1,iCIdx,  jl0[nRowSkip * iLocalConstraint + 1] );
						B.set( 6 * index0 + 2,iCIdx,  jl0[nRowSkip * iLocalConstraint + 2] );

						// angular part
						B.set( iCIdx, 6 * index0 + 3, ja0[nRowSkip * iLocalConstraint + 0] );
						B.set( iCIdx, 6 * index0 + 4, ja0[nRowSkip * iLocalConstraint + 1] );
						B.set( iCIdx, 6 * index0 + 5, ja0[nRowSkip * iLocalConstraint + 2] );
						// transpose
						B.set( 6 * index0 + 3,iCIdx,  ja0[nRowSkip * iLocalConstraint + 0] );
						B.set( 6 * index0 + 4,iCIdx,  ja0[nRowSkip * iLocalConstraint + 1] );
						B.set( 6 * index0 + 5,iCIdx,  ja0[nRowSkip * iLocalConstraint + 2] );
					}

				if ( ( index1 != -1 ) && ( index1 != index0 ) )
					for ( iLocalConstraint = 0; iLocalConstraint < info1.m_numConstraintRows; iLocalConstraint++ )
					{
						// column index of iLocalConstraint
						iCIdx = iStartColWb + iConstraint + iLocalConstraint;
						// linear part
						B.set( iCIdx, 6 * index1 + 0, -jl0[nRowSkip * iLocalConstraint + 0] );
						B.set( iCIdx, 6 * index1 + 1, -jl0[nRowSkip * iLocalConstraint + 1] );
						B.set( iCIdx, 6 * index1 + 2, -jl0[nRowSkip * iLocalConstraint + 2] );
						// transpose
						B.set( 6 * index1 + 0,iCIdx,  -jl0[nRowSkip * iLocalConstraint + 0] );
						B.set( 6 * index1 + 1,iCIdx,  -jl0[nRowSkip * iLocalConstraint + 1] );
						B.set( 6 * index1 + 2,iCIdx,  -jl0[nRowSkip * iLocalConstraint + 2] );

						// angular part
						B.set( iCIdx, 6 * index1 + 3, ja1[nRowSkip * iLocalConstraint + 0] );
						B.set( iCIdx, 6 * index1 + 4, ja1[nRowSkip * iLocalConstraint + 1] );
						B.set( iCIdx, 6 * index1 + 5, ja1[nRowSkip * iLocalConstraint + 2] );
						// transpose
						B.set( 6 * index1 + 3,iCIdx,  ja1[nRowSkip * iLocalConstraint + 0] );
						B.set( 6 * index1 + 4,iCIdx,  ja1[nRowSkip * iLocalConstraint + 1] );
						B.set( 6 * index1 + 5,iCIdx,  ja1[nRowSkip * iLocalConstraint + 2] );
					}

				for ( iLocalConstraint = 0; iLocalConstraint < info1.m_numConstraintRows; iLocalConstraint++ )
				{
					// column index of iLocalConstraint
					iCIdx = iStartColWb + iConstraint + iLocalConstraint;
					b[ iCIdx ] = -jErr[iLocalConstraint * nRowSkip];
					l[iCIdx]   = lLimits[iLocalConstraint * nRowSkip];
					u[iCIdx]   = uLimits[iLocalConstraint * nRowSkip];
				}

				iConstraint += iLocalConstraint;
			}
		}

		btAssert( iConstraint == numConstraints );

		// Wn part
		// Matrix<double> Wn( 6* numMovableBodies , numContacts );

		int                                iContact    = 0;
		int                                iStartColWn = iStartColWb + numConstraints;
		int                                iStartColWf = iStartColWn + numContacts;
		int                                iStartColE  = iStartColWf + numContacts * m_frictionDirs;
		btAlignedObjectArray < btVector3 > bodyFrictions;
		bodyFrictions.resize( m_frictionDirs );

		// printf("num manifolds: [%d]  ", numManifolds);

		btScalar totalPen = 0.0;

		for ( i = 0; i < numManifolds; i++ )
		{
			btPersistentManifold* contactManifold = getDispatcher()->getManifoldByIndexInternal( i );
			btCollisionObject   * obA = static_cast < btCollisionObject * > ( contactManifold->getBody0() );
			btCollisionObject   * obB = static_cast < btCollisionObject * > ( contactManifold->getBody1() );

			btRigidBody* body0 = btRigidBody::upcast( obA );
			btRigidBody* body1 = btRigidBody::upcast( obB );
			int        index0  = body0 ? body0->m_bodyIdx : -1;
			int        index1  = body1 ? body1->m_bodyIdx : -1;

			// contactManifold->refreshContactPoints(obA->getWorldTransform(),obB->getWorldTransform());

			// printf("num contacts: [%d].\n",numContacts);

			for ( int j = 0; j < contactManifold->getNumContacts(); j++ )
			{
				btManifoldPoint & pt = contactManifold->getContactPoint( j );

				// Reconstruct contact point
				// pt.m_positionWorldOnB = body1->getWorldTransform()*pt.m_localPointB;
				// pt.m_positionWorldOnA = body0->getWorldTransform()*pt.m_localPointA;
				// pt.m_distance1 = (pt.m_positionWorldOnA -  pt.m_positionWorldOnB).dot(pt.m_normalWorldOnB);

				const btVector3 & normal = pt.m_normalWorldOnB.normalized();
				btVector3       w;
				btVector3       n;
				int             i_cf = 0;

				totalPen = totalPen + pt.getDistance();

				// printf("contact %d =[%f,%f,%f] [%f].\n",iContact,pt.m_localPointB.x(),
				// pt.m_localPointB.y(),pt.m_localPointB.z(),pt.getDistance());
				// debugDrawSphere(5.0f,obA->getWorldTransform(),btVector3(0,1.0f,0));
				// getDebugDrawer()->drawLine(pt.getPositionWorldOnA(),pt.getPositionWorldOnB(),btVector3(0.0f,1.0f,0.0f));
				// getDebugDrawer()->drawContactPoint(pt.getPositionWorldOnB(),pt.m_normalWorldOnB,
				// pt.getDistance(),pt.getLifeTime(),btVector3(0,255,0));

				// Fill wrench matrix at block (iContact,index0*6)

				// U
				if ( m_frictionDirs )
				{
					B.set( iStartColE + iContact, iStartColWn + iContact, pt.m_combinedFriction ); // 1.0 is coefficient of friction

					for ( i_cf = 0; i_cf < m_frictionDirs; i_cf++ )
					{
						B.set( iStartColE + iContact, iStartColWf + iContact * m_frictionDirs + i_cf, -1.0 );
						B.set( iStartColWf + iContact * m_frictionDirs + i_cf, iStartColE + iContact,  1.0 );

						// printf ("pt.m_combinedFriction = %f.\n",pt.m_combinedFriction);
					}
				}

				btVector3 d1; // other 2 axes of contact frame
				btVector3 d2; // the first axis is the contact normal

				// Create the contact frame by 3 vectors (n,n cross v = d, n cross d)
				if ( normal.x() > btScalar( 0.5 ) )
				{
					d1 = normal.cross( btVector3( 0,1,0 ) );
					d1.normalize();

					d2 = normal.cross( d1 );
					d2.normalize();
				}
				else
				{
					d1 = normal.cross( btVector3( 1,0,0 ) );
					d1.normalize();

					d2 = normal.cross( d1 );
					d2.normalize();
				}

				// Now normal,d1,d2 should be the 3 axes of the contact frame.
				btMatrix3x3 rotContactToBody( d1.x(),d2.x(),normal.x()
						,d1.y(),d2.y(),normal.y()
						,d1.z(),d2.z(),normal.z() );

				if ( index0 != -1 )
				{
					btTransform & trans0 = body0->getWorldTransform();
					// w = trans0.frameTransform(pt.m_localPointA).cross(normal);
					w = ( trans0.getBasis() * ( pt.m_localPointA ) ).cross( normal );
					btTransform & invTrans0 = trans0.inverse();
					// Wn
					B.set( iStartColWn + iContact, 6 * index0 + 0, normal.x() );
					B.set( iStartColWn + iContact, 6 * index0 + 1, normal.y() );
					B.set( iStartColWn + iContact, 6 * index0 + 2, normal.z() );
					B.set( iStartColWn + iContact, 6 * index0 + 3, w.x() );
					B.set( iStartColWn + iContact, 6 * index0 + 4, w.y() );
					B.set( iStartColWn + iContact, 6 * index0 + 5, w.z() );

					// Wn transpose
					B.set( 6 * index0 + 0, iStartColWn + iContact, normal.x() );
					B.set( 6 * index0 + 1, iStartColWn + iContact, normal.y() );
					B.set( 6 * index0 + 2, iStartColWn + iContact, normal.z() );
					B.set( 6 * index0 + 3, iStartColWn + iContact, w.x() );
					B.set( 6 * index0 + 4, iStartColWn + iContact, w.y() );
					B.set( 6 * index0 + 5, iStartColWn + iContact, w.z() );

					if ( m_frictionDirs )
						// Wf
						for ( i_cf = 0; i_cf < m_frictionDirs; i_cf++ )
						{
							// bodyFrictions[i_cf] = rotContactToBody * m_localFrictionDirs[i_cf];
							bodyFrictions[i_cf] = rotContactToBody * m_localFrictionDirs[i_cf];
							// Wf at iContact, friction direction i_cf
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index0 + 0, bodyFrictions[i_cf].x() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index0 + 1, bodyFrictions[i_cf].y() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index0 + 2, bodyFrictions[i_cf].z() );
							// transpose
							B.set( 6 * index0 + 0,iStartColWf + iContact * m_frictionDirs + i_cf, bodyFrictions[i_cf].x() );
							B.set( 6 * index0 + 1,iStartColWf + iContact * m_frictionDirs + i_cf, bodyFrictions[i_cf].y() );
							B.set( 6 * index0 + 2,iStartColWf + iContact * m_frictionDirs + i_cf, bodyFrictions[i_cf].z() );

							// Debug drawing
							// getDebugDrawer()->drawLine(pt.m_positionWorldOnA,pt.m_positionWorldOnA + 2.0*bodyFrictions[i_cf],btVector3(0,255,0));

							const btVector3 & fx = ( pt.getPositionWorldOnA() - body0->getCenterOfMassPosition() ).cross( bodyFrictions[i_cf] );

							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index0 + 3, fx.x() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index0 + 4, fx.y() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index0 + 5, fx.z() );
							// transpose
							B.set( 6 * index0 + 3,iStartColWf + iContact * m_frictionDirs + i_cf, fx.x() );
							B.set( 6 * index0 + 4,iStartColWf + iContact * m_frictionDirs + i_cf, fx.y() );
							B.set( 6 * index0 + 5,iStartColWf + iContact * m_frictionDirs + i_cf, fx.z() );
						}
				}

				if ( index1 != -1 )
				{
					btTransform & trans1    = body1->getWorldTransform();
					btTransform & invTrans1 = trans1.inverse();
					// w = trans1.frameTransform(pt.m_localPointB).cross(normal);
					w = ( trans1.getBasis() * pt.m_localPointB ).cross( normal );
					// Wn
					B.set( iStartColWn + iContact, 6 * index1 + 0, -normal.x() );
					B.set( iStartColWn + iContact, 6 * index1 + 1, -normal.y() );
					B.set( iStartColWn + iContact, 6 * index1 + 2, -normal.z() );
					B.set( iStartColWn + iContact, 6 * index1 + 3, -w.x() );
					B.set( iStartColWn + iContact, 6 * index1 + 4, -w.y() );
					B.set( iStartColWn + iContact, 6 * index1 + 5, -w.z() );

					// Wn transpose
					B.set( 6 * index1 + 0, iStartColWn + iContact, -normal.x() );
					B.set( 6 * index1 + 1, iStartColWn + iContact, -normal.y() );
					B.set( 6 * index1 + 2, iStartColWn + iContact, -normal.z() );
					B.set( 6 * index1 + 3, iStartColWn + iContact, -w.x() );
					B.set( 6 * index1 + 4, iStartColWn + iContact, -w.y() );
					B.set( 6 * index1 + 5, iStartColWn + iContact, -w.z() );

					if ( m_frictionDirs )
						for ( i_cf = 0; i_cf < m_frictionDirs; i_cf++ )
						{
							bodyFrictions[i_cf] = rotContactToBody * m_localFrictionDirs[i_cf];
							// Wf at iContact, friction direction i_cf
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index1 + 0, -bodyFrictions[i_cf].x() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index1 + 1, -bodyFrictions[i_cf].y() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index1 + 2, -bodyFrictions[i_cf].z() );
							// transpose
							B.set( 6 * index1 + 0,iStartColWf + iContact * m_frictionDirs + i_cf, -bodyFrictions[i_cf].x() );
							B.set( 6 * index1 + 1,iStartColWf + iContact * m_frictionDirs + i_cf, -bodyFrictions[i_cf].y() );
							B.set( 6 * index1 + 2,iStartColWf + iContact * m_frictionDirs + i_cf, -bodyFrictions[i_cf].z() );

							// Debug drawing
							// getDebugDrawer()->drawLine(pt.m_positionWorldOnB,pt.m_positionWorldOnB + 2.0*bodyFrictions[i_cf],btVector3(255,0,0));

							// bodyFrictions[i_cf] = pt.m_localPointB.cross(bodyFrictions[i_cf]);
							const btVector3 & fx = ( pt.getPositionWorldOnB() - body1->getCenterOfMassPosition() ).cross( -bodyFrictions[i_cf] );

							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index1 + 3, fx.x() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index1 + 4, fx.y() );
							B.set( iStartColWf + iContact * m_frictionDirs + i_cf, 6 * index1 + 5, fx.z() );
							// transpose
							B.set( 6 * index1 + 3,iStartColWf + iContact * m_frictionDirs + i_cf, fx.x() );
							B.set( 6 * index1 + 4,iStartColWf + iContact * m_frictionDirs + i_cf, fx.y() );
							B.set( 6 * index1 + 5,iStartColWf + iContact * m_frictionDirs + i_cf, fx.z() );
						}
				}

				//// check if penetration
				if ( pt.getDistance() < 0.0 )
					// then we should use m_stablizationFraction
					b[ iStartColWn + iContact ] = m_stablizationFraction * pt.getDistance() / timeStep;
				else
					// positive distance can ignore m_stablizationFraction
					b[ iStartColWn + iContact ] = pt.getDistance() / timeStep;

				// printf(" %f \n", m_stablizationFraction);

				iContact++;
			}

			// you can un-comment out this line, and then all points are removed
			//contactManifold->clearManifold();
		}

		double* z = new double[n];  /* Solution vector                     */

		// Build l and u
		double big = 1e20;

		for ( i = 0; i < numMovableBodies * 6; i++ )
		{
			l[i] = -big;
			u[i] = big;
		}

		for ( i = numMovableBodies * 6 + numConstraints; i < n; i++ )
		{
			l[i] = 0;
			u[i] = big;
		}

		for ( i = 0; i < n; i++ )
			// u[i] = big;
			z[i] = 0.0f;

		// logging the whole matrix system
#ifdef _SAVE_LCP_
		{
			char _sName[250];
			sprintf(_sName,"m_%06d.dat",m_LCPFrame);

			std::string sName(_sName);
			// Matrix
			B.save(sName.c_str());

			// outZ
			{
				sName[0] = 'i';
				saveVectorToFile(z, n, sName.c_str());
			}

			// lower
			{
				sName[0] = 'l';
				saveVectorToFile(l, n, sName.c_str());
			}

			// upper
			{
				sName[0] = 'u';
				saveVectorToFile(u, n, sName.c_str());
			}

			// rhs
			{
				sName[0] = 'b';
				saveVectorToFile(&b[0], n, sName.c_str());
			}
		}
#endif

		bool result = SolveMCP( z, l, u, B, &b[0] );

		if ( !result )
			printf( "Fail to solve LCP.\n" );
		else
		{
			// Now updated body velocity
			for ( i = 0; i < numMovableBodies; i++ )
			{
				btRigidBody* pBody = btRigidBody::upcast( bodyList[bodyIndexArray[i]] );
				pBody->setLinearVelocity( btVector3( btScalar( z[6 * i] ), btScalar( z[6 * i + 1] ), btScalar( z[6 * i + 2] ) ) );
				pBody->setAngularVelocity( btVector3( btScalar( z[6 * i + 3] ), btScalar( z[6 * i + 4] ), btScalar( z[6 * i + 5] ) ) );

				// pBody->clearForces();

				// printf("Old pos: [%f %f %f]\n",pBody->getWorldTransform().getOrigin().x()
				// ,pBody->getWorldTransform().getOrigin().y()
				// ,pBody->getWorldTransform().getOrigin().z());
			}

#ifdef _SAVE_LCP_

			// outZ
			{
				char _sName[250];
				sprintf(_sName,"r_%06d.dat",m_LCPFrame);

				std::string sName(_sName);

				saveVectorToFile(z, n, sName.c_str());
			}

#endif
		}

#ifdef _SAVE_LCP_
		{
			m_LCPFrame++;
		}
#endif
		// // Print
		// printf("\n Contact normal forces = [");
		// for(i = numMovableBodies*6 + numConstraints ; i <  numMovableBodies*6 + numConstraints +numContacts; i++)
		// {
		// printf(" %f",z[i]);
		// }
		// printf("]\n");

		// printf("\n Friction forces");
		// for (iContact = 0; iContact < numContacts; iContact++)
		// {
		// printf("\n F[%d] = [",iContact);
		// for(i = 0 ; i <  m_frictionDirs; i++)
		// {
		// printf(" %f",z[numMovableBodies*6 + numConstraints + iContact*m_frictionDirs + i]);
		// }
		// printf("]\n");
		// }
		// printf("\n");

		// B.print("B = ");
		// b.print("b = ");

		// B.print(1);

		// printf("z = [");
		// for(i = 0; i < n; i++)
		// {
		// printf("%f ",z[i]);
		// }
		// printf("]\n");

		// printf("l = [");
		// for(i = 0; i < n; i++)
		// {
		// printf("%f ",l[i]);
		// }
		// printf("]\n");

		// printf("Contact stats: [%d] contacts, [%f] average distances.\n",numContacts,totalPen);

		delete[] u;
		delete[] l;
		delete[] z;
	}
	else
	{
		// /apply gravity, predict motion
		// predictUnconstraintMotion(timeStep);
	}

	/***********************************************************************
	                      End
	***********************************************************************/

	#else // if USE_ST_METHOD

	calculateSimulationIslands();

	getSolverInfo().m_timeStep = timeStep;

	// /solve contact and other joint constraints
	solveConstraints( getSolverInfo() );

	// /CallbackTriggers();
	#endif // if USE_ST_METHOD

	// /integrate transforms
	integrateTransforms( timeStep );

	// /update vehicle simulation
	updateActions( timeStep );

	updateActivationState( timeStep );

	if ( 0 != m_internalTickCallback )
		( *m_internalTickCallback )( this, timeStep );
}

/***********************************************************************
   Additional stuffs for ST time-stepper
***********************************************************************/
void btST2World::generateLocalFrictions()
{
	m_localFrictionDirs.resize( m_frictionDirs );

	btScalar  deg  = btScalar( 0.0 );
	btScalar  step = SIMD_2_PI / btScalar( m_frictionDirs );
	btVector3 v;
	int       i = 0;

	for ( i = 0; i < m_frictionDirs; i++ )
	{
		v.setValue( btCos( deg ), btSin( deg ), btScalar( 0.0 ) );
		m_localFrictionDirs[i] = v;
		deg += step;
	}
} // generateLocalFrictions

#ifdef _SAVE_LCP_
void saveVectorToFile(double* v,int n,const char* file)
{
	FILE* f;
	errno_t err = fopen_s( &f, file, "w+" );

	int i = 0;

	// size
	//fprintf(f, "%d\n",n);

	for (i=0; i < n; i++)
	{
		fprintf(f, "%lg\n",v[i]);
	}

	fclose(f);
}
#endif
/***********************************************************************
   End
***********************************************************************/

void	btST2World::setGravity(const btVector3& gravity)
{
	m_gravity = gravity;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
		{
			body->setGravity(gravity);
		}
	}
}

btVector3 btST2World::getGravity () const
{
	return m_gravity;
}

void	btST2World::addCollisionObject(btCollisionObject* collisionObject,short int collisionFilterGroup,short int collisionFilterMask)
{
	btCollisionWorld::addCollisionObject(collisionObject,collisionFilterGroup,collisionFilterMask);
}

void	btST2World::removeCollisionObject(btCollisionObject* collisionObject)
{
	btRigidBody* body = btRigidBody::upcast(collisionObject);
	if (body)
		removeRigidBody(body);
	else
		btCollisionWorld::removeCollisionObject(collisionObject);
}

void	btST2World::removeRigidBody(btRigidBody* body)
{
	m_nonStaticRigidBodies.remove(body);
	btCollisionWorld::removeCollisionObject(body);
}

void	btST2World::addRigidBody(btRigidBody* body)
{
	if (!body->isStaticOrKinematicObject() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		} else
		{
			body->setActivationState(ISLAND_SLEEPING);
		}

		bool isDynamic = !(body->isStaticObject() || body->isKinematicObject());
		short collisionFilterGroup = isDynamic? short(btBroadphaseProxy::DefaultFilter) : short(btBroadphaseProxy::StaticFilter);
		short collisionFilterMask = isDynamic? 	short(btBroadphaseProxy::AllFilter) : 	short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

		addCollisionObject(body,collisionFilterGroup,collisionFilterMask);
	}
}

void	btST2World::addRigidBody(btRigidBody* body, short group, short mask)
{
	if (!body->isStaticOrKinematicObject() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		}
		else
		{
			body->setActivationState(ISLAND_SLEEPING);
		}
		addCollisionObject(body,group,mask);
	}
}

void	btST2World::updateActions(btScalar timeStep)
{
	BT_PROFILE("updateActions");

	for ( int i=0;i<m_actions.size();i++)
	{
		m_actions[i]->updateAction( this, timeStep);
	}
}

void	btST2World::updateActivationState(btScalar timeStep)
{
	BT_PROFILE("updateActivationState");

	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body)
		{
			body->updateDeactivation(timeStep);

			if (body->wantsSleeping())
			{
				if (body->isStaticOrKinematicObject())
				{
					body->setActivationState(ISLAND_SLEEPING);
				} else
				{
					if (body->getActivationState() == ACTIVE_TAG)
						body->setActivationState( WANTS_DEACTIVATION );
					if (body->getActivationState() == ISLAND_SLEEPING)
					{
						body->setAngularVelocity(btVector3(0,0,0));
						body->setLinearVelocity(btVector3(0,0,0));
					}
				}
			} else
			{
				if (body->getActivationState() != DISABLE_DEACTIVATION)
					body->setActivationState( ACTIVE_TAG );
			}
		}
	}
}

void	btST2World::addConstraint(btTypedConstraint* constraint,bool disableCollisionsBetweenLinkedBodies)
{
	m_constraints.push_back(constraint);
	if (disableCollisionsBetweenLinkedBodies)
	{
		constraint->getRigidBodyA().addConstraintRef(constraint);
		constraint->getRigidBodyB().addConstraintRef(constraint);
	}
}

void	btST2World::removeConstraint(btTypedConstraint* constraint)
{
	m_constraints.remove(constraint);
	constraint->getRigidBodyA().removeConstraintRef(constraint);
	constraint->getRigidBodyB().removeConstraintRef(constraint);
}

void	btST2World::addAction(btActionInterface* action)
{
	m_actions.push_back(action);
}

void	btST2World::removeAction(btActionInterface* action)
{
	m_actions.remove(action);
}

void	btST2World::addVehicle(btActionInterface* vehicle)
{
	addAction(vehicle);
}

void	btST2World::removeVehicle(btActionInterface* vehicle)
{
	removeAction(vehicle);
}

void	btST2World::addCharacter(btActionInterface* character)
{
	addAction(character);
}

void	btST2World::removeCharacter(btActionInterface* character)
{
	removeAction(character);
}

SIMD_FORCE_INLINE	int	btGetConstraintIslandId(const btTypedConstraint* lhs)
{
	int islandId;

	const btCollisionObject& rcolObj0 = lhs->getRigidBodyA();
	const btCollisionObject& rcolObj1 = lhs->getRigidBodyB();
	islandId= rcolObj0.getIslandTag()>=0?rcolObj0.getIslandTag():rcolObj1.getIslandTag();
	return islandId;
}

class btSortConstraintOnIslandPredicate
{
public:

	bool operator() ( const btTypedConstraint* lhs, const btTypedConstraint* rhs )
	{
		int rIslandId0,lIslandId0;
		rIslandId0 = btGetConstraintIslandId(rhs);
		lIslandId0 = btGetConstraintIslandId(lhs);
		return lIslandId0 < rIslandId0;
	}
};

void	btST2World::solveConstraints(btContactSolverInfo& solverInfo)
{
	BT_PROFILE("solveConstraints");

	struct InplaceSolverIslandCallback : public btSimulationIslandManager::IslandCallback
	{
		btContactSolverInfo&	m_solverInfo;
		btConstraintSolver*		m_solver;
		btTypedConstraint**		m_sortedConstraints;
		int						m_numConstraints;
		btIDebugDraw*			m_debugDrawer;
		btStackAlloc*			m_stackAlloc;
		btDispatcher*			m_dispatcher;

		btAlignedObjectArray<btCollisionObject*> m_bodies;
		btAlignedObjectArray<btPersistentManifold*> m_manifolds;
		btAlignedObjectArray<btTypedConstraint*> m_constraints;

		InplaceSolverIslandCallback(
			btContactSolverInfo& solverInfo,
			btConstraintSolver*	solver,
			btTypedConstraint** sortedConstraints,
			int	numConstraints,
			btIDebugDraw*	debugDrawer,
			btStackAlloc*			stackAlloc,
			btDispatcher* dispatcher)
			:m_solverInfo(solverInfo),
			m_solver(solver),
			m_sortedConstraints(sortedConstraints),
			m_numConstraints(numConstraints),
			m_debugDrawer(debugDrawer),
			m_stackAlloc(stackAlloc),
			m_dispatcher(dispatcher)
		{
		}

		InplaceSolverIslandCallback& operator=(InplaceSolverIslandCallback& other)
		{
			btAssert(0);
			(void)other;
			return *this;
		}
		virtual	void	ProcessIsland(btCollisionObject** bodies,int numBodies,btPersistentManifold**	manifolds,int numManifolds, int islandId)
		{
			if (islandId<0)
			{
				if (numManifolds + m_numConstraints)
				{
					///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
					m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,&m_sortedConstraints[0],m_numConstraints,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
				}
			} else
			{
				//also add all non-contact constraints/joints for this island
				btTypedConstraint** startConstraint = 0;
				int numCurConstraints = 0;
				int i;

				//find the first constraint for this island
				for (i=0;i<m_numConstraints;i++)
				{
					if (btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
					{
						startConstraint = &m_sortedConstraints[i];
						break;
					}
				}
				//count the number of constraints in this island
				for (;i<m_numConstraints;i++)
				{
					if (btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
					{
						numCurConstraints++;
					}
				}

				if (m_solverInfo.m_minimumSolverBatchSize<=1)
				{
					///only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
					if (numManifolds + numCurConstraints)
					{
						m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,startConstraint,numCurConstraints,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
					}
				} else
				{
					for (i=0;i<numBodies;i++)
						m_bodies.push_back(bodies[i]);
					for (i=0;i<numManifolds;i++)
						m_manifolds.push_back(manifolds[i]);
					for (i=0;i<numCurConstraints;i++)
						m_constraints.push_back(startConstraint[i]);
					if ((m_constraints.size()+m_manifolds.size())>m_solverInfo.m_minimumSolverBatchSize)
					{
						processConstraints();
					} else
					{
						//printf("deferred\n");
					}
				}
			}
		}
		void	processConstraints()
		{
			if (m_manifolds.size() + m_constraints.size()>0)
			{
				m_solver->solveGroup( &m_bodies[0],m_bodies.size(), &m_manifolds[0], m_manifolds.size(), &m_constraints[0], m_constraints.size() ,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
			}
			m_bodies.resize(0);
			m_manifolds.resize(0);
			m_constraints.resize(0);
		}
	};

	//sorted version of all btTypedConstraint, based on islandId
	btAlignedObjectArray<btTypedConstraint*>	sortedConstraints;
	sortedConstraints.resize( m_constraints.size());
	int i;
	for (i=0;i<getNumConstraints();i++)
	{
		sortedConstraints[i] = m_constraints[i];
	}

	//	btAssert(0);

	sortedConstraints.quickSort(btSortConstraintOnIslandPredicate());

	btTypedConstraint** constraintsPtr = getNumConstraints() ? &sortedConstraints[0] : 0;

	InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver, constraintsPtr,sortedConstraints.size(),	m_debugDrawer,m_stackAlloc,m_dispatcher1);

	m_constraintSolver->prepareSolve(getCollisionWorld()->getNumCollisionObjects(), getCollisionWorld()->getDispatcher()->getNumManifolds());

	/// solve all the constraints for this island
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(),getCollisionWorld(),&solverCallback);

	solverCallback.processConstraints();

	m_constraintSolver->allSolved(solverInfo, m_debugDrawer, m_stackAlloc);
}

void	btST2World::calculateSimulationIslands()
{
	BT_PROFILE("calculateSimulationIslands");

	getSimulationIslandManager()->updateActivationState(getCollisionWorld(),getCollisionWorld()->getDispatcher());

	{
		int i;
		int numConstraints = int(m_constraints.size());
		for (i=0;i< numConstraints ; i++ )
		{
			btTypedConstraint* constraint = m_constraints[i];

			const btRigidBody* colObj0 = &constraint->getRigidBodyA();
			const btRigidBody* colObj1 = &constraint->getRigidBodyB();

			if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
				((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
			{
				if (colObj0->isActive() || colObj1->isActive())
				{
					getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(),
						(colObj1)->getIslandTag());
				}
			}
		}
	}

	//Store the island id in each body
	getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());
}

class btClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
	btCollisionObject* m_me;
	btScalar m_allowedPenetration;
	btOverlappingPairCache* m_pairCache;
	btDispatcher* m_dispatcher;

public:
	btClosestNotMeConvexResultCallback (btCollisionObject* me,const btVector3& fromA,const btVector3& toA,btOverlappingPairCache* pairCache,btDispatcher* dispatcher) :
	  btCollisionWorld::ClosestConvexResultCallback(fromA,toA),
		  m_me(me),
		  m_allowedPenetration(0.0f),
		  m_pairCache(pairCache),
		  m_dispatcher(dispatcher)
	  {
	  }

	  virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	  {
		  if (convexResult.m_hitCollisionObject == m_me)
			  return 1.0f;

		  //ignore result if there is no contact response
		  if(!convexResult.m_hitCollisionObject->hasContactResponse())
			  return 1.0f;

		  btVector3 linVelA,linVelB;
		  linVelA = m_convexToWorld-m_convexFromWorld;
		  linVelB = btVector3(0,0,0);//toB.getOrigin()-fromB.getOrigin();

		  btVector3 relativeVelocity = (linVelA-linVelB);
		  //don't report time of impact for motion away from the contact normal (or causes minor penetration)
		  if (convexResult.m_hitNormalLocal.dot(relativeVelocity)>=-m_allowedPenetration)
			  return 1.f;

		  return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	  }

	  virtual bool needsCollision(btBroadphaseProxy* proxy0) const
	  {
		  //don't collide with itself
		  if (proxy0->m_clientObject == m_me)
			  return false;

		  ///don't do CCD when the collision filters are not matching
		  if (!ClosestConvexResultCallback::needsCollision(proxy0))
			  return false;

		  btCollisionObject* otherObj = (btCollisionObject*) proxy0->m_clientObject;

		  //call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
		  if (m_dispatcher->needsResponse(m_me,otherObj))
		  {
#if 0
			  ///don't do CCD when there are already contact points (touching contact/penetration)
			  btAlignedObjectArray<btPersistentManifold*> manifoldArray;
			  btBroadphasePair* collisionPair = m_pairCache->findPair(m_me->getBroadphaseHandle(),proxy0);
			  if (collisionPair)
			  {
				  if (collisionPair->m_algorithm)
				  {
					  manifoldArray.resize(0);
					  collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
					  for (int j=0;j<manifoldArray.size();j++)
					  {
						  btPersistentManifold* manifold = manifoldArray[j];
						  if (manifold->getNumContacts()>0)
							  return false;
					  }
				  }
			  }
#endif
			  return true;
		  }

		  return false;
	  }
};

///internal debugging variable. this value shouldn't be too high
int gNumST2ClampedCcdMotions=0;

//#include "stdio.h"
void	btST2World::integrateTransforms(btScalar timeStep)
{
	BT_PROFILE("integrateTransforms");
	btTransform predictedTrans;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{
			body->predictIntegratedTransform(timeStep, predictedTrans);
			/*
			btScalar squareMotion = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin()).length2();

			if (body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion)
			{
				BT_PROFILE("CCD motion clamping");
				if (body->getCollisionShape()->isConvex())
				{
					gNumST2ClampedCcdMotions++;

					btClosestNotMeConvexResultCallback sweepResults(body,body->getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase()->getOverlappingPairCache(),getDispatcher());
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					btSphereShape tmpSphere(body->getCcdSweptSphereRadius());//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());

					sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask  = body->getBroadphaseProxy()->m_collisionFilterMask;

					convexSweepTest(&tmpSphere,body->getWorldTransform(),predictedTrans,sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f))
					{
						body->setHitFraction(sweepResults.m_closestHitFraction);
						body->predictIntegratedTransform(timeStep*body->getHitFraction(), predictedTrans);
						body->setHitFraction(0.f);
						body->setLinearVelocity(btVector3(0,0.1,0));

						//							printf("clamped integration to hit fraction = %f\n",fraction);
					}
				}
			}
			*/

			body->proceedToTransform( predictedTrans);
		}
	}
}

void	btST2World::predictUnconstraintMotion(btScalar timeStep)
{
	BT_PROFILE("predictUnconstraintMotion");
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (!body->isStaticOrKinematicObject())
		{
			body->integrateVelocities( timeStep);
			//damping
			body->applyDamping(timeStep);

			body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
		}
	}
}

void	btST2World::startProfiling(btScalar timeStep)
{
	(void)timeStep;

#ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif //BT_NO_PROFILE
}

void btST2World::debugDrawConstraint(btTypedConstraint* constraint)
{
	bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
	btScalar dbgDrawSize = constraint->getDbgDrawSize();
	if(dbgDrawSize <= btScalar(0.f))
	{
		return;
	}

	switch(constraint->getConstraintType())
	{
	case POINT2POINT_CONSTRAINT_TYPE:
		{
			btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)constraint;
			btTransform tr;
			tr.setIdentity();
			btVector3 pivot = p2pC->getPivotInA();
			pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot;
			tr.setOrigin(pivot);
			getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			// that ideally should draw the same frame
			pivot = p2pC->getPivotInB();
			pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot;
			tr.setOrigin(pivot);
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
		}
		break;
	case HINGE_CONSTRAINT_TYPE:
		{
			btHingeConstraint* pHinge = (btHingeConstraint*)constraint;
			btTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			btScalar minAng = pHinge->getLowerLimit();
			btScalar maxAng = pHinge->getUpperLimit();
			if(minAng == maxAng)
			{
				break;
			}
			bool drawSect = true;
			if(minAng > maxAng)
			{
				minAng = btScalar(0.f);
				maxAng = SIMD_2_PI;
				drawSect = false;
			}
			if(drawLimits)
			{
				btVector3& center = tr.getOrigin();
				btVector3 normal = tr.getBasis().getColumn(2);
				btVector3 axis = tr.getBasis().getColumn(0);
				getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btVector3(0,0,0), drawSect);
			}
		}
		break;
	case CONETWIST_CONSTRAINT_TYPE:
		{
			btConeTwistConstraint* pCT = (btConeTwistConstraint*)constraint;
			btTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			if(drawLimits)
			{
				//const btScalar length = btScalar(5);
				const btScalar length = dbgDrawSize;
				static int nSegments = 8*4;
				btScalar fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)(nSegments-1)/btScalar(nSegments);
				btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
				pPrev = tr * pPrev;
				for (int i=0; i<nSegments; i++)
				{
					fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)i/btScalar(nSegments);
					btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
					pCur = tr * pCur;
					getDebugDrawer()->drawLine(pPrev, pCur, btVector3(0,0,0));

					if (i%(nSegments/8) == 0)
						getDebugDrawer()->drawLine(tr.getOrigin(), pCur, btVector3(0,0,0));

					pPrev = pCur;
				}
				btScalar tws = pCT->getTwistSpan();
				btScalar twa = pCT->getTwistAngle();
				bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btScalar(0.f));
				if(useFrameB)
				{
					tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
				}
				else
				{
					tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
				}
				btVector3 pivot = tr.getOrigin();
				btVector3 normal = tr.getBasis().getColumn(0);
				btVector3 axis1 = tr.getBasis().getColumn(1);
				getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, btVector3(0,0,0), true);
			}
		}
		break;
	case D6_SPRING_CONSTRAINT_TYPE:
	case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
			btTransform tr = p6DOF->getCalculatedTransformA();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = p6DOF->getCalculatedTransformB();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			if(drawLimits)
			{
				tr = p6DOF->getCalculatedTransformA();
				const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
				btVector3 up = tr.getBasis().getColumn(2);
				btVector3 axis = tr.getBasis().getColumn(0);
				btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
				btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
				btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
				btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
				getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0,0,0));
				axis = tr.getBasis().getColumn(1);
				btScalar ay = p6DOF->getAngle(1);
				btScalar az = p6DOF->getAngle(2);
				btScalar cy = btCos(ay);
				btScalar sy = btSin(ay);
				btScalar cz = btCos(az);
				btScalar sz = btSin(az);
				btVector3 ref;
				ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
				ref[1] = -sz*axis[0] + cz*axis[1];
				ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
				tr = p6DOF->getCalculatedTransformB();
				btVector3 normal = -tr.getBasis().getColumn(0);
				btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
				btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
				if(minFi > maxFi)
				{
					getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
				}
				else if(minFi < maxFi)
				{
					getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
				}
				tr = p6DOF->getCalculatedTransformA();
				btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
				btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
				getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0,0,0));
			}
		}
		break;
	case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
			btTransform tr = pSlider->getCalculatedTransformA();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = pSlider->getCalculatedTransformB();
			if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			if(drawLimits)
			{
				btTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
				btVector3 li_min = tr * btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
				btVector3 li_max = tr * btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
				getDebugDrawer()->drawLine(li_min, li_max, btVector3(0, 0, 0));
				btVector3 normal = tr.getBasis().getColumn(0);
				btVector3 axis = tr.getBasis().getColumn(1);
				btScalar a_min = pSlider->getLowerAngLimit();
				btScalar a_max = pSlider->getUpperAngLimit();
				const btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
				getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0,0,0), true);
			}
		}
		break;
	default :
		break;
	}
	return;
}

void	btST2World::setConstraintSolver(btConstraintSolver* solver)
{
	if (m_ownsConstraintSolver)
	{
		btAlignedFree( m_constraintSolver);
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
}

btConstraintSolver* btST2World::getConstraintSolver()
{
	return m_constraintSolver;
}

int		btST2World::getNumConstraints() const
{
	return int(m_constraints.size());
}
btTypedConstraint* btST2World::getConstraint(int index)
{
	return m_constraints[index];
}
const btTypedConstraint* btST2World::getConstraint(int index) const
{
	return m_constraints[index];
}

void	btST2World::serializeRigidBodies(btSerializer* serializer)
{
	int i;
	//serialize all collision objects
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() & btCollisionObject::CO_RIGID_BODY)
		{
			int len = colObj->calculateSerializeBufferSize();
			btChunk* chunk = serializer->allocate(len,1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk,structType,BT_RIGIDBODY_CODE,colObj);
		}
	}

	for (i=0;i<m_constraints.size();i++)
	{
		btTypedConstraint* constraint = m_constraints[i];
		int size = constraint->calculateSerializeBufferSize();
		btChunk* chunk = serializer->allocate(size,1);
		const char* structType = constraint->serialize(chunk->m_oldPtr,serializer);
		serializer->finalizeChunk(chunk,structType,BT_CONSTRAINT_CODE,constraint);
	}
}

void	btST2World::serialize(btSerializer* serializer)
{
	serializer->startSerialization();

	serializeRigidBodies(serializer);

	serializeCollisionObjects(serializer);

	serializer->finishSerialization();
}