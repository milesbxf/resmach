package me.mb.resmach.ui;

/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.Physics;

import org.lwjgl.LWJGLException;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class Visualiser extends DemoApplication {

	private Physics physics;

	public Visualiser(IGL gl) {
		super(gl);
	}
	
	public void initialise(Physics physics) {
		this.physics = physics;
		this.dynamicsWorld = physics.getDynamicsWorld();
		setCameraDistance(20f);
		clientResetScene();
	}
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// simple dynamics world doesn't handle fixed-time-stepping
		float ms = getDeltaTimeMicroseconds();
		
		physics.update(ms);

		renderme();

		//glFlush();
		//glutSwapBuffers();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();

		// optional but useful: debug drawing to detect problems
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}

		//glFlush();
		//glutSwapBuffers();
	}

	
	
	public static void createVisualiser(Physics physics) throws LWJGLException {
		Visualiser ccdDemo = new Visualiser(LWJGL.getGL());
		ccdDemo.initialise(physics);		
		physics.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

		String[] args = new String[] {};
		LWJGL.main(args, 800, 600, "Bullet Physics Demo. http://bullet.sf.net", ccdDemo);
	}

	@Override
	public void initPhysics() throws Exception {
		// TODO Auto-generated method stub
		
	}
	
}
