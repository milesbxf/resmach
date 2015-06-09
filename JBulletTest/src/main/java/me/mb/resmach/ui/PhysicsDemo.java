package me.mb.resmach.ui;

import java.util.List;
import java.util.Random;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.Physics;
import me.mb.resmach.physsim.RobotBody;
import me.mb.resmach.physsim.Updatable;
import me.mb.resmach.physsim.Physics.Axis;

import org.lwjgl.LWJGLException;

import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.google.common.collect.Lists;

public class PhysicsDemo implements Updatable {

	Physics physics;
	HingeConstraint constraints[] = new HingeConstraint[8];
	float motorCommands[] = new float[8];
	
	public PhysicsDemo(Physics physics) {
		this.physics = physics;
	}

	public static void main(String[] args) throws LWJGLException {
		Physics physics = new Physics();
		
		PhysicsDemo demo = new PhysicsDemo(physics);
//		demo.createBody();
		demo.createRobot();
		
//		physics.setUpdatable(demo);
		Visualiser.createVisualiser(physics);
	}

	public void createRobot() {

		List<Integer> attachmentLocs = Lists.newArrayList(0, 1, 0, 3, 0, 5,
				0, 7);
		List<Float> perimeterLocs = Lists.newArrayList(0.25f, 0.5f, 0.5f,
				0.5f, 0.75f, 0.5f, 1f, 0.5f);
		
		RobotBody body = new RobotBody(physics,Lists.newArrayList(0,1,0,3,0,5,0,7),Lists.newArrayList(0.25f,0.5f,0.5f,0.5f,0.75f,0.5f,1f,0.5f));
//		RobotBody body = new RobotBody(physics,Lists.newArrayList(0,1,0,3),Lists.newArrayList(0.375f,0.5f,0.625f,0.5f));
	}
	
//	public void createBody() {
//		for (int i = 0; i < motorCommands.length; i++) {
//			motorCommands[i] = new Random().nextFloat() * (float)Math.PI - (float)(Math.PI/2);
//		}
//		
//		RigidBody mainBody = physics.createBox(0f, 1f, 0f, 2f, 0.2f, 2f);
//
//		float deg45 = (float) (45 * Math.PI / 180);
//
//		RigidBody legUpperN = physics.createCylinder(Axis.Z, 0, 1, 1.5f, 0.2f,
//				1f);
//		
//		constraints[0] = physics.createHinge(mainBody, legUpperN, new Vector3f(1,0,0), new Vector3f(1,0,0), new Vector3f(
//				0, 0, 0.75f), new Vector3f(0, 0, -0.75f), deg45);
//
//		RigidBody legLowerN = physics.createCylinder(Axis.Y, 0f, 0.5f, 2f,
//				0.2f, 1f);
//		constraints[1] = physics.createHinge(legUpperN, legLowerN, new Vector3f(1,0,0), new Vector3f(1,0,0), new Vector3f(
//				0, 0, 0.6f), new Vector3f(0, 0.5f, 0f), deg45);
//
//		RigidBody legUpperS = physics.createCylinder(Axis.Z, 0, 1, -1.5f, 0.2f,
//				1f);
//		constraints[2] = physics.createHinge(mainBody, legUpperS, new Vector3f(1,0,0), new Vector3f(1,0,0), new Vector3f(
//				0, 0, -0.75f), new Vector3f(0, 0, 0.75f), deg45);
//
//		RigidBody legLowerS = physics.createCylinder(Axis.Y, 0f, 0.5f, -2f,
//				0.2f, 1f);
//		constraints[3] = physics.createHinge(legUpperS, legLowerS, new Vector3f(1,0,0), new Vector3f(1,0,0), new Vector3f(
//				0, 0, -0.6f), new Vector3f(0, 0.5f, 0f), deg45);
//
//		RigidBody legUpperW = physics.createCylinder(Axis.X, 1.5f, 1, 0, 0.2f,
//				1f);
//		constraints[4] = physics.createHinge(mainBody, legUpperW, new Vector3f(0,0,1), new Vector3f(0,0,1), new Vector3f(
//				0.75f, 0, 0), new Vector3f(-0.75f, 0, 0), deg45);
//
//		RigidBody legLowerW = physics.createCylinder(Axis.Y, 2f, 0.5f, 0, 0.2f,
//				1f);
//		constraints[5] = physics.createHinge(legUpperW, legLowerW, new Vector3f(0,0,1), new Vector3f(0,0,1), new Vector3f(
//				0.6f, 0, 0), new Vector3f(0, 0.5f, 0f), deg45);
//
//		RigidBody legUpperE = physics.createCylinder(Axis.X, -1.5f, 1, 0, 0.2f,
//				1f);
//		constraints[6] = physics.createHinge(mainBody, legUpperE, new Vector3f(0,0,1), new Vector3f(0,0,1), new Vector3f(
//				-0.75f, 0, 0), new Vector3f(0.75f, 0, 0), deg45);
//
//		RigidBody legLowerE = physics.createCylinder(Axis.Y, -2f, 0.5f, 0,
//				0.2f, 1f);
//		constraints[7] = physics.createHinge(legUpperE, legLowerE, new Vector3f(0,0,1), new Vector3f(0,0,1),new Vector3f(
//				-0.6f, 0, 0), new Vector3f(0, 0.5f, 0f), deg45);
//
//	}

	public void update(float deltaTime) {
		for (int i = 0; i < constraints.length; i++) {
			physics.actuateJoint(constraints[i], (float)(motorCommands[i]), 0, deltaTime);
//			motorCommands[i] = 0.1f * (float)(Math.random() - 0.5);
		}		
	}
	
	private float degToRad(float degrees) {
		return degrees * (float)Math.PI/180;
	}
}
