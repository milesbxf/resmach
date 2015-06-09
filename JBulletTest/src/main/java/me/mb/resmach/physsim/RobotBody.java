package me.mb.resmach.physsim;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.GeomUtils;
import me.mb.resmach.physsim.util.PhysicsUtils;
import me.mb.resmach.physsim.util.PhysicsUtils.Axis;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.google.common.collect.Lists;

public class RobotBody {

	private final List<BodyPart> bodyParts;
	private final List<BodyJoint> joints;
	private final Physics physics;
	private Vector3f[] bodyLocs;
	private Vector3f[] jointLocsA,jointLocsB;
	private float[] orientations;

	public RobotBody(Physics physics, List<Integer> attachmentLocs,
			List<Float> perimeterLocs) {
		bodyParts = new ArrayList<>();
		joints = new ArrayList<>();
		this.physics = physics;

		createMainBody(2, 0.5f, 2);

		calculateSegmentAndJointPositions(attachmentLocs, perimeterLocs);

		for (int i = 0; i < attachmentLocs.size(); i++) {
			createSegment(bodyLocs[i], orientations[i], 0.2f, 1f);
		}

		for (int i = 0; i < attachmentLocs.size(); i++) {
			createJoint(attachmentLocs.get(i), i + 1);
		}
	}

	/**
	 * Creates the main body of the robot.
	 * 
	 * @param length
	 *            Size in X axis.
	 * @param height
	 *            Size in Y axis.
	 * @param width
	 *            Size in Z axis.
	 */
	private void createMainBody(float length, float height, float width) {
		BoxBodyPart mainBody = new BoxBodyPart(new Vector3f(0, 1, 0),
				new Vector3f(length, height, width));
		bodyParts.add(mainBody);
		physics.getDynamicsWorld().addRigidBody(mainBody.body);
	}

	private void createSegment(Vector3f position, float rotation, float radius,
			float length) {
		CylinderBodyPart segment = new CylinderBodyPart(new Vector3f(), rotation,
				radius, length);
		bodyParts.add(segment);
		physics.getDynamicsWorld().addRigidBody(segment.body);
	}

	private void createJoint(int bodyAindex, int bodyBindex) {
		BodyPart bodyA = bodyParts.get(bodyAindex), bodyB = bodyParts
				.get(bodyBindex);
		float rotationA, rotationB;
//		Vector3f locA = bodyA.transformWorldToLocal(jointLocs[bodyBindex - 1]),
//		locB = bodyB.transformWorldToLocal(jointLocs[bodyBindex - 1]);
		
		Vector3f locA=jointLocsA[bodyBindex - 1],locB=jointLocsB[bodyBindex - 1];
		rotationB = orientations[bodyBindex - 1];
		
		BodyJoint joint = new BodyJoint(bodyA, bodyB, rotationB, rotationB,
				locA,locB);
		physics.getDynamicsWorld().addConstraint(joint.getConstraint());
	}

	/**
	 * Calculates attachment points and positions of the segments.
	 * 
	 * @param attachmentIndices
	 *            List of indices that each segment connects to. E.g. for
	 *            {0,1,4}, the first segment connects to the main body(0), the
	 *            second connects to the first segment (1) and the third
	 *            connects to the fourth(4).
	 * @param perimeterLocs
	 *            List of points at which each segment connects to its
	 *            connecting body, in the range [0,1]. This maps onto an angle
	 *            which also determines orientation of the segment.
	 */
	private void calculateSegmentAndJointPositions(
			List<Integer> attachmentIndices, List<Float> perimeterLocs) {

		int n = attachmentIndices.size();
		bodyLocs = new Vector3f[n];
		jointLocsA = new Vector3f[n];
		jointLocsB = new Vector3f[n];
		orientations = new float[n];

		for (int i = 0; i < n; i++) {

			int index = attachmentIndices.get(i);
			float angBodyB = perimeterLocs.get(i) * 2 * PhysicsUtils.PIf;
			float halfDist = 0.6f;

			if (index == 0) { // if this segment attaches to the main body
				Vector2f pt2D = GeomUtils.getPointOnRectPerimeter(2, 2,
						angBodyB);
				orientations[i] = angBodyB;
				
				jointLocsA[i] = new Vector3f(pt2D.x, 0, pt2D.y);
				jointLocsB[i] = new Vector3f(-halfDist, 0, 0);
				getBodyLocs()[i] = new Vector3f(getJointLocs()[i]);
				getBodyLocs()[i].add(new Vector3f(halfDist
						* (float) Math.cos(angBodyB), 0f, -halfDist
						* (float) Math.sin(angBodyB)));
			} else {
				float angBodyA = perimeterLocs.get(index - 1) * 2
						* PhysicsUtils.PIf;
				Vector2f pt2D = GeomUtils.getPointOnRectPerimeter(1f, 0.2f,
						angBodyB);

				orientations[i] = GeomUtils.PIf + (angBodyA - angBodyB);

				float cosx = (float) Math.cos(angBodyA - angBodyB), sinx = (float) Math
						.sin(angBodyA - angBodyB);
				jointLocsA[i] = new Vector3f(pt2D.x,0,pt2D.y);
				jointLocsB[i] = new Vector3f(-halfDist,0,0);
//				getJointLocs()[i] = new Vector3f(pt2D.x * cosx + pt2D.y * sinx,
//						0, -pt2D.x * sinx + pt2D.y * cosx);
				// now translate relative to body
//				getJointLocs()[i].add(getBodyLocs()[index - 1]);
				getBodyLocs()[i] = new Vector3f(jointLocsB[i]);
				getBodyLocs()[i].add(new Vector3f(halfDist
						* (float) Math.cos(angBodyA), 0f, -halfDist
						* (float) Math.sin(angBodyA)));
			}
		}
	}

	public List<BodyPart> getBodyParts() {
		return bodyParts;
	}

	Vector3f[] getBodyLocs() {
		return bodyLocs;
	}

	Vector3f[] getJointLocs() {
		return jointLocsA;
	}

}
