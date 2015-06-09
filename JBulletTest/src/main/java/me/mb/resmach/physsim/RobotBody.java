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


		for (int i = 0; i < attachmentLocs.size(); i++) {
			createSegment(0.2f, 1f);
		}

		calculateSegmentAndJointPositions(attachmentLocs, perimeterLocs);
//		for (int i = 0; i < attachmentLocs.size(); i++) {
//			createJoint(attachmentLocs.get(i), i + 1);
//			
//		}
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

		for (int bodyBindex = 0; bodyBindex < n; bodyBindex++) {
			float xWidth,yWidth,rotation;
			
			int bodyAindex = attachmentIndices.get(bodyBindex);
			float bodyBrotation = perimeterLocs.get(bodyBindex) * 2 * PhysicsUtils.PIf;

			BodyPart bodyA = bodyParts.get(bodyAindex);
			BodyPart bodyB = bodyParts.get(bodyBindex+1);
			Vector3f pivotA,pivotB;
			if(bodyAindex == 0) {
				xWidth=3f;yWidth=3f;rotation = bodyBrotation;
				Vector2f pivotPt = GeomUtils.getPointOnRectPerimeter(xWidth, yWidth,
						bodyBrotation);
				pivotA = new Vector3f(pivotPt.x,0,pivotPt.y);
			} else {
				xWidth = 1.2f;yWidth=0.4f;
				Vector2f pivotPt = GeomUtils.getPointOnRectPerimeter(xWidth, yWidth,
						bodyBrotation);
				rotation = GeomUtils.PIf + perimeterLocs.get(bodyAindex-1)* 2 * PhysicsUtils.PIf - bodyBrotation ;
				pivotA = new Vector3f(0,pivotPt.x,pivotPt.y);
			}
			
			
			
			pivotB = new Vector3f(0,0.5f,0);
						
			BodyJoint joint = new BodyJoint(bodyA,bodyB,rotation,rotation,pivotA,pivotB);
			
			joints.add(joint);

			physics.getDynamicsWorld().addConstraint(joint.getConstraint());
			
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
