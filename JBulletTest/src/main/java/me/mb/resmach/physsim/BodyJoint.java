package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.PhysicsUtils.Axis;
import me.mb.resmach.physsim.util.GeomUtils;
import me.mb.resmach.physsim.util.PhysicsUtils;

import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;

public class BodyJoint {

	private final BodyPart bodyA, bodyB;
	private final HingeConstraint constraint;
	private float currentDesiredAngle = 0;

	public BodyJoint(BodyPart bodyA, BodyPart bodyB, float rotationA,
			float rotationB, Vector3f pivotA, Vector3f pivotB) {
		this.bodyA = bodyA;
		this.bodyB = bodyB;

		Vector3f axisA = new Vector3f((float) Math.sin(rotationA), 0,
				(float) Math.cos(rotationA)), axisB = new Vector3f(
				(float) Math.sin(rotationB), 0, (float) Math.cos(rotationB));
		
		axisA.absolute();axisB.absolute();
		
//		axisA = new Vector3f(0,0,1);axisB = new Vector3f(0,0,1);
		
		constraint = new HingeConstraint(bodyA.body, bodyB.body, pivotA, pivotB,
				axisA,axisB);
		
		constraint.setLimit(GeomUtils.degToRad(-45), GeomUtils.degToRad(45));

	}

	public TypedConstraint getConstraint() {
		return constraint;
	}
	
	public void actuate(Physics physics,float deltaTime) {
		physics.actuateJoint(constraint, currentDesiredAngle, 0, deltaTime);
	}
	
	public void sendMotorCommand(float deltaAngle) {
		currentDesiredAngle += deltaAngle;
		currentDesiredAngle %= (2 * GeomUtils.PIf);
	}
	
	public float getCurrentAngle() {
		return constraint.getHingeAngle();
	}
}
