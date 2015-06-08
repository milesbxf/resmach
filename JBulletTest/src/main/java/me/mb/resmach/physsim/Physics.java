package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeX;
import com.bulletphysics.collision.shapes.CylinderShapeZ;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;

/**
 * An interface between the application and the jBullet physics engine. 
 * 
 * @author Miles Bryant
 */
public class Physics {

	
	// keep the collision shapes, for deletion/cleanup
	private ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
	private DynamicsWorld dynamicsWorld;

	private Updatable updatable;
	
	public Physics() {
		initPhysics();
		createGround();
	}
	
	private void initPhysics() {

		// collision configuration contains default setup for memory, collision
		// setup
		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can
		// use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		// the default constraint solver. For parallel processing you can use a
		// different solver (see Extras/BulletMultiThreaded)
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;

		// TODO: needed for SimpleDynamicsWorld
		// sol.setSolverMode(sol.getSolverMode() &
		// ~SolverMode.SOLVER_CACHE_FRIENDLY.getMask());

		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase,
				solver, collisionConfiguration);

		dynamicsWorld.setGravity(new Vector3f(0f, -10f, 0f));

	}

	private void createGround() {
		CollisionShape groundShape = new BoxShape(new Vector3f(50f, 1f, 50f));

		collisionShapes.add(groundShape);

		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(0, -1f, 0);

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
		dynamicsWorld.addRigidBody(body);
	}


//	public RigidBody createBox(float posX, float posY, float posZ,
//			float length, float height, float width) {
//		CollisionShape colShape = new BoxShape(new Vector3f(length / 2,
//				height / 2, width / 2));
//		return createBody(posX, posY, posZ, colShape);
//	}

	public static enum Axis {
		X, Y, Z
	}

	public RigidBody createCylinder(Axis axis, float posX, float posY,
			float posZ, float radius, float length) {
		CollisionShape colShape;
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

	private RigidBody createBody(float posX, float posY, float posZ,
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

		dynamicsWorld.addRigidBody(body);
		body.setActivationState(RigidBody.ISLAND_SLEEPING);
		collisionShapes.add(shape);
		return body;
	}

	public HingeConstraint createHinge(RigidBody bodyA, RigidBody bodyB, Vector3f axisA,
			Vector3f axisB, Vector3f pivotA, Vector3f pivotB, float angleLimit) {
		
		HingeConstraint hc = new HingeConstraint(bodyA, bodyB, pivotA, pivotB,
				axisA, axisB);

		hc.setLimit(-angleLimit, angleLimit);

		dynamicsWorld.addConstraint(hc);
		return hc;
	}

	public void update(float deltaTime) {
		if(updatable != null) {
			updatable.update(deltaTime);
		}
		
		// step the simulation
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(deltaTime / 1000000f);
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
		}
	}

	public void actuateJoint(HingeConstraint joint,float desiredAngle, float jointOffset, float timeStep) {
		float actualAngle = joint.getHingeAngle();
		float diffAng = desiredAngle-actualAngle;
		joint.enableAngularMotor(true, diffAng, 0.5f);
		
	}
	
	public DynamicsWorld getDynamicsWorld() {
		return dynamicsWorld;
	}
	
	public Updatable getUpdatable() {
		return updatable;
	}
	public void setUpdatable(Updatable updatable) {
		this.updatable = updatable;
	}
	
}
