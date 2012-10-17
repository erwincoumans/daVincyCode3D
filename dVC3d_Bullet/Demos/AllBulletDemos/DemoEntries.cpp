/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "DemoEntries.h"

#include "../BasicDemo/BasicDemo.h"
#include "../DynamicControlDemo/MotorDemo.h"
#include "../RagdollDemo/RagdollDemo.h"
#include "../ConstraintDemo/ConstraintDemo.h"

#include "GLDebugFont.h"

#include "GlutStuff.h"//OpenGL stuff


extern int gNumAlignedAllocs;
extern int gNumAlignedFree;
extern int gTotalBytesAlignedAllocs;

class btEmptyDebugDemo : public GlutDemoApplication
{
public:
	btEmptyDebugDemo()
	{

	}

	virtual void clientMoveAndDisplay()
	{
				
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

		float xOffset = 10.f;
		float yStart = 20.f;
		float yIncr = 20.f;
		char buf[124];


		glColor3f(0, 0, 0);

		setOrthographicProjection();

		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"gNumAlignedAllocs= %d",gNumAlignedAllocs);
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;

		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"gNumAlignedFree= %d",gNumAlignedFree);
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;

		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"# alloc-free = %d",gNumAlignedAllocs-gNumAlignedFree);
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"gTotalBytesAlignedAllocs = %d",gTotalBytesAlignedAllocs);
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
#endif //BT_DEBUG_MEMORY_ALLOCATIONS

	glFlush();
	glutSwapBuffers();
			
	}

	virtual	void initPhysics() {}

	static DemoApplication* Create()
	{
		btEmptyDebugDemo* demo = new btEmptyDebugDemo();
		demo->myinit();
		return demo;
	}

};


btDemoEntry g_demoEntries[] =
{

	{"Basic Demo", BasicDemo::Create},	
	{"Dynamic Control Demo",MotorDemo::Create},
	{"ConstraintDemo",ConstraintDemo::Create},
	{"Ragdoll Demo",RagdollDemo::Create},
	{"MemoryLeak Checker",btEmptyDebugDemo::Create},	
	{0, 0}
};


