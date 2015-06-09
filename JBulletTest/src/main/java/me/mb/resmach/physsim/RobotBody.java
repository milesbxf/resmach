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

	public RobotBody(Physics physics, List<Integer> attachmentLocs,
			List<Float> perimeterLocs) {
		bodyParts = new ArrayList<>();
		joints = new ArrayList<>();
		this.physics = physics;

		createMainBody(2, 0.5f, 2);

		for (int i = 0; i < attachmentLocs.size(); i++) {
			createSegment(0.2f, 1f);
		}

		createJoints(attachmentLocs, perimeterLocs);
	}

	private void createSegment(float radius, float length) {
		CylinderBodyPart segment = new CylinderBodyPart(new Vector3f(), 0,
				radius, length);
		bodyParts.add(segment);
		physics.getDynamicsWorld().addRigidBody(segment.body);
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
	private void createJoints(List<Integer> attachmentIndices,
			List<Float> perimeterLocs) {

		int n = attachmentIndices.size(); // number of segments to join

		for (int bodyBindex = 0; bodyBindex < n; bodyBindex++) {

			float xWidth, // width of rectangle for perimeter calculation
			yWidth, // height
			rotation; // orientation of the joint to calculate constraining axis

			int bodyAindex = attachmentIndices.get(bodyBindex);
			float bodyBrotation = perimeterLocs.get(bodyBindex) * 2
					* PhysicsUtils.PIf;

			// get body parts to join
			BodyPart bodyA = bodyParts.get(bodyAindex);

			// body parts has an extra element at beginning (the main body) so
			// index
			// should be incremented by one
			BodyPart bodyB = bodyParts.get(bodyBindex + 1);

			Vector3f pivotA, pivotB; // pivot points of joint (in local
										// coordinates)

			if (bodyAindex == 0) { // attaching segment to main body

				xWidth = 3f;
				yWidth = 3f;
				rotation = bodyBrotation;

				Vector2f pivotPt = GeomUtils.getPointOnRectPerimeter(xWidth,
						yWidth, bodyBrotation);

				pivotA = new Vector3f(pivotPt.x, 0, pivotPt.y); // pivot is in
																// XZ axis

			} else { // attaching segment to another segment
				xWidth = 1.2f;
				yWidth = 0.4f;
				rotation = GeomUtils.PIf + perimeterLocs.get(bodyAindex - 1)
						* 2 * PhysicsUtils.PIf - bodyBrotation;

				Vector2f pivotPt = GeomUtils.getPointOnRectPerimeter(xWidth,
						yWidth, bodyBrotation);

				pivotA = new Vector3f(0, pivotPt.x, pivotPt.y); // pivot in YZ
																// axis
			}

			pivotB = new Vector3f(0, 0.5f, 0); // second pivot is always in Y
												// axis

			// create the joint and add it to the world
			BodyJoint joint = new BodyJoint(bodyA, bodyB, rotation, rotation,
					pivotA, pivotB);
			joints.add(joint);
			physics.getDynamicsWorld().addConstraint(joint.getConstraint());

		}
	}

	public List<BodyPart> getBodyParts() {
		return bodyParts;
	}

	public List<BodyJoint> getJoints() {
		return joints;
	}
}
