package me.mb.resmach.physsim;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.GeomUtils;
import me.mb.resmach.physsim.util.PhysicsUtils;
import me.mb.resmach.physsim.util.PhysicsUtils.Axis;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.google.common.collect.Lists;

public class RobotBody {

	private final List<BodyPart> bodyParts;
	private final Physics physics;
	Vector3f[] bodyLocs, jointLocs;

	public RobotBody(Physics physics, List<Integer> attachmentLocs,
			List<Float> perimeterLocs) {
		bodyParts = new ArrayList<>();
		this.physics = physics;

		createMainBody(2, 0.5f, 2);
		for (int i = 0; i < attachmentLocs.size(); i++) {
			createSegment(0.2f, 1f);
		}

		calculateAttachmentLocations(attachmentLocs, perimeterLocs);
	}

	/**
	 * Creates the main body of the robot.
	 * 
	 * @param length
	 *            Size in X axis.
	 * @param height
	 *            Size in Y axis.index
	 * @param width
	 *            Size in Z axis.
	 */
	private void createMainBody(float length, float height, float width) {
		BoxBodyPart mainBody = new BoxBodyPart(new Vector3f(0, 1, 0),
				new Vector3f(length, height, width), 1);
		bodyParts.add(mainBody);
		physics.getDynamicsWorld().addRigidBody(mainBody.body);
	}

	private void createSegment(float radius, float length) {
		CylinderBodyPart segment = new CylinderBodyPart(new Vector3f(0, 1, 0),
				Axis.X, radius, length);
		bodyParts.add(segment);
		physics.getDynamicsWorld().addRigidBody(segment.body);
	}

	private void calculateAttachmentLocations(List<Integer> attachmentLocs,
			List<Float> perimeterLocs) {
		int n = attachmentLocs.size();

		bodyLocs = new Vector3f[n];
		jointLocs = new Vector3f[n];
		float[] orientations = new float[n];

		for (int i = 0; i < n; i++) {

			// calculate attachment point
			int index = attachmentLocs.get(i);
			float halfDist = 0.5f;
			float angBodyB = perimeterLocs.get(i) * 2 * PhysicsUtils.PIf;
			float angBodyA = 0;
			if (index == 0) {
				Vector2f point2D = GeomUtils.getPointOnRectPerimeter(2, 2,
						angBodyB);
				orientations[i] = angBodyB;
				jointLocs[i] = new Vector3f(point2D.x, 1, point2D.y);
				bodyLocs[i] = new Vector3f(jointLocs[i]);
				bodyLocs[i].add(new Vector3f(halfDist
						* (float) Math.cos(angBodyB), 0f, -halfDist
						* (float) Math.sin(angBodyB)));
			} else {
				angBodyA = perimeterLocs.get(index-1) * 2 * PhysicsUtils.PIf;
				Vector2f pt2D = GeomUtils.getPointOnRectPerimeter(1f, 0.2f,
						angBodyB);
				
				float cosx = (float)Math.cos(angBodyA - angBodyB),sinx = (float)Math.sin(angBodyA - angBodyB);
				
				jointLocs[i] = new Vector3f(
						pt2D.x * cosx + pt2D.y * sinx,
						0,
						-pt2D.x * sinx + pt2D.y * cosx
						);
				//now translate relative to body 
				jointLocs[i].add(bodyLocs[index-1]);
				bodyLocs[i] = new Vector3f(jointLocs[i]);
				bodyLocs[i].add(new Vector3f(halfDist
						* (float) Math.cos(angBodyA), 0f, -halfDist
						* (float) Math.sin(angBodyA)));
			}

			// now project half the length of the segment on the perimeter angle
			// from the AP


		}
	}

		
	public List<BodyPart> getBodyParts() {
		return bodyParts;
	}

}
