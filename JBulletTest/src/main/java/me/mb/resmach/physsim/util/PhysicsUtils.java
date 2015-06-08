package me.mb.resmach.physsim.util;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.Physics.Axis;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeX;
import com.bulletphysics.collision.shapes.CylinderShapeZ;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

public class PhysicsUtils {
	
	
	public static float PIf = (float)Math.PI;
	
	/**
	 * Creates the ground as a box shape with specified side length to be added to a DynamicWorld.
	 * 
	 * @param sideLength Length (in metres) of the sides of the ground. Objects falling outside of the bounds of the ground will fall off.
	 * @return RigidBody for adding to a DynamicWorld.
	 */
	public static RigidBody createGround(float sideLength) {
		// create a few basic rigid bodies
		CollisionShape groundShape = new BoxShape(new Vector3f(sideLength, 1f, sideLength));

		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(0, -0.5f, 0);

		// We can also use DemoApplication::localCreateRigidBody, but for
		// clarity it is provided here:
		float mass = 0f;

		// rigidbody is dynamic if and only if mass is non zero, otherwise
		// static
		boolean isDynamic = (mass != 0f);

		Vector3f localInertia = new Vector3f(0, 0, 0);
		if (isDynamic) {
			groundShape.calculateLocalInertia(mass, localInertia);
		}

		// using motionstate is recommended, it provides interpolation
		// capabilities, and only synchronizes 'active' objects
		DefaultMotionState myMotionState = new DefaultMotionState(
				groundTransform);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass,
				myMotionState, groundShape, localInertia);
		RigidBody body = new RigidBody(rbInfo);

		// add the body to the dynamics world
		return body;
	}

	/**
	 * Creates a box shape.
	 * @param posX X position.
	 * @param posY Y position.
	 * @param posZ Z position.
	 * @param length Length (size in X dimension)
	 * @param height Height (size in Y dimension)
	 * @param width Width (size in Z dimension)
	 * @return RigidBody to add to a DynamicWorld
	 */
	public static RigidBody createBox(float posX, float posY, float posZ,
			float length, float height, float width) {
		//create a new BoxShape with the half-lengths
		CollisionShape colShape = new BoxShape(new Vector3f(length / 2,
				height / 2, width / 2));
		//let createBody handle the transforming
		return createBody(posX, posY, posZ, colShape);
	}
	
	public static enum Axis {
		X, Y, Z
	}
	
	/**
	 * Creates a cylindrical shape.
	 * @param axis Axis cylinder is aligned on.
	 * @param posX X position.
	 * @param posY Y position.
	 * @param posZ Z position.
	 * @param radius Radius of cylinder.
	 * @param length Length of cylinder.
	 * @return RigidBody for adding to a DynamicsWorld
	 */
	public static RigidBody createCylinder(Axis axis, float posX, float posY,
			float posZ, float radius, float length) {
		CollisionShape colShape;
		
		//Select a cylinder class based on axis
		if (axis == Axis.Y)
			colShape = new CylinderShape(new Vector3f(radius, length / 2,
					radius));
		else if (axis == Axis.X)
			colShape = new CylinderShapeX(new Vector3f(length / 2, radius,
					radius));
		else if (axis == Axis.Z)
			colShape = new CylinderShapeZ(new Vector3f(radius, radius,
					length / 2));
		else
			throw new IllegalArgumentException("Axis must be X,Y or Z");

		return createBody(posX, posY, posZ, colShape);
	}

	/**
	 * Creates a RigidBody at specified position with specified shape.
	 * @param posX X position.
	 * @param posY Y position.
	 * @param posZ Z position.
	 * @param shape CollisionShape.
	 * @return RigidBody to add to DynamicsWorld.
	 */
	public static RigidBody createBody(float posX, float posY, float posZ,
			CollisionShape shape) {

		float mass = 1f;

		// rigidbody is dynamic if and only if mass is non zero, otherwise
		// static
		boolean isDynamic = (mass != 0f);

		Vector3f localInertia = new Vector3f(0, 0, 0);
		if (isDynamic) {
			shape.calculateLocalInertia(mass, localInertia);
		}

		// Create Dynamic Objects
		Transform transform = new Transform();
		transform.setIdentity();

		transform.origin.set(posX, posY, posZ);
		// using motionstate is recommended, it provides
		// interpolation capabilities, and only synchronizes
		// 'active' objects
		DefaultMotionState myMotionState = new DefaultMotionState(transform);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass,
				myMotionState, shape, localInertia);
		RigidBody body = new RigidBody(rbInfo);
		body.setActivationState(RigidBody.ISLAND_SLEEPING);
		
		return body;
	}

	public static RigidBody createCylinder(float radius, float length) {
		// TODO Auto-generated method stub
		return null;
	}
}
