package me.mb.resmach.physsim;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.GeomUtils;
import mockit.Capturing;
import mockit.Cascading;
import mockit.Expectations;
import mockit.Injectable;
import mockit.Mocked;
import mockit.NonStrictExpectations;
import mockit.Verifications;

import org.junit.Test;

import com.bulletphysics.dynamics.DynamicsWorld;
import com.google.common.collect.Lists;

public class RobotBodyTest {


	@Cascading
	Physics physics;
	@Cascading
	DynamicsWorld world;

	private List<Integer> attachmentLocs = Lists.newArrayList(0, 1, 0, 3, 0, 5,
			0, 7);
	private List<Float> perimeterLocs = Lists.newArrayList(0.25f, 0.5f, 0.5f,
			0.5f, 0.75f, 0.5f, 1f, 0.5f);

	@Test
	public void testCreatesMainBodyAtOrigin() {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(0, 1, 0), result = body.getBodyParts()
				.get(0).position;

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsFirstJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};		
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);		
		new JointVerifications(
				0,
				body.getBodyParts().get(0), 
				body.getBodyParts().get(1), 
				GeomUtils.degToRad(90), 
				GeomUtils.degToRad(90), 
				new Vector3f(0,0,-1), 
				new Vector3f(0.5f,0,0)
				);
		
	}
	@Test
	public void createsSecondJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				1,
				body.getBodyParts().get(1), 
				body.getBodyParts().get(2), 
				GeomUtils.degToRad(90), 
				GeomUtils.degToRad(90), 
				new Vector3f(-0.5f,0,0), 
				new Vector3f(0.5f,0,0)
				);
	}
	

	@Test
	public void createsThirdJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				2,
				body.getBodyParts().get(0), 
				body.getBodyParts().get(3), 
				GeomUtils.degToRad(180), 
				GeomUtils.degToRad(180), 
				new Vector3f(-1f,0,0), 
				new Vector3f(0.5f,0,0)
				);
	}

	@Test
	public void createsFourthJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				3,
				body.getBodyParts().get(3), 
				body.getBodyParts().get(4), 
				GeomUtils.degToRad(180), 
				GeomUtils.degToRad(180), 
				new Vector3f(-0.5f,0,0), 
				new Vector3f(0.5f,0,0)
				);
	}
	

	@Test
	public void createsFifthJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				4,
				body.getBodyParts().get(0), 
				body.getBodyParts().get(5), 
				GeomUtils.degToRad(270), 
				GeomUtils.degToRad(270), 
				new Vector3f(0,0,1f), 
				new Vector3f(0.5f,0,0)
				);
	}
	

	@Test
	public void createsSixthJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				5,
				body.getBodyParts().get(5), 
				body.getBodyParts().get(6), 
				GeomUtils.degToRad(270), 
				GeomUtils.degToRad(270), 
				new Vector3f(-0.5f,0,0), 
				new Vector3f(0.5f,0,0)
				);
	}


	@Test
	public void createsSeventhJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				6,
				body.getBodyParts().get(0), 
				body.getBodyParts().get(7), 
				GeomUtils.degToRad(360), 
				GeomUtils.degToRad(360), 
				new Vector3f(1f,0,0), 
				new Vector3f(0.5f,0,0)
				);
	}
	

	@Test
	public void createsEighthJoint(@Mocked BodyJoint joint,
			@Mocked final BodyPart bodyPart) throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		final RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		new JointVerifications(
				7,
				body.getBodyParts().get(7), 
				body.getBodyParts().get(8), 
				GeomUtils.degToRad(360), 
				GeomUtils.degToRad(360), 
				new Vector3f(-0.5f,0,0), 
				new Vector3f(0.5f,0,0)
				);
	}
	
	public static class JointVerifications extends Verifications {
		JointVerifications(int iteration,BodyPart bodyA, BodyPart bodyB, float rotationA, float rotationB, Vector3f pivotA, Vector3f pivotB) {
			List<BodyPart> actBodyA = new ArrayList<>(),actBodyB = new ArrayList<>();
			List<Float> actRotationA = new ArrayList<>(),actRotationB = new ArrayList<>();
			List<Vector3f> actPivotA = new ArrayList<>(),actPivotB = new ArrayList<>();
			new BodyJoint(
					withCapture(actBodyA), 
					withCapture(actBodyB),
					withCapture(actRotationA), 
					withCapture(actRotationB), 
					withCapture(actPivotA),
					withCapture(actPivotB)
					);
			assertEquals(String.format("Expected %s and got %s",bodyA,actBodyA.get(iteration)),actBodyA.get(iteration),bodyA);
			assertEquals(String.format("Expected %s and got %s",bodyB,actBodyB.get(iteration)),actBodyB.get(iteration),bodyB);
			assertEquals(String.format("Expected %f and got %f",rotationA,actRotationA.get(iteration)),actRotationA.get(iteration),rotationA,0.001f);
			assertEquals(String.format("Expected %f and got %f",rotationB,actRotationB.get(iteration)),actRotationB.get(iteration),rotationB,0.001f);
			assertTrue(String.format("Expected %s and got %s",pivotA,actPivotA.get(iteration)),actPivotA.get(iteration).epsilonEquals(pivotA, 0.001f));
			assertTrue(String.format("Expected %s and got %s",pivotB,actPivotB.get(iteration)),actPivotB.get(iteration).epsilonEquals(pivotB, 0.001f));
		}
	}
	
	
	//
	// @Test
	// public void createsSecondJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	// Vector3f expected = new Vector3f(0f, 1f, -2f), result =
	// body.getJointLocs()[1];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	// }
	//
	// @Test
	// public void createsThirdJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	//
	// Vector3f expected = new Vector3f(-1.5f, 1f, 0f), result =
	// body.getBodyLocs()[2];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	//
	// }
	//
	// @Test
	// public void createsFourthJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	// Vector3f expected = new Vector3f(-2f, 1f, 0f), result =
	// body.getJointLocs()[3];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	// }
	//
	// @Test
	// public void createsFifthJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	// Vector3f expected = new Vector3f(0f, 1f, 1f), result =
	// body.getJointLocs()[4];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	// }
	//
	// @Test
	// public void createsSixthJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	// Vector3f expected = new Vector3f(0f, 1f, 2f), result =
	// body.getJointLocs()[5];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	// }
	//
	// @Test
	// public void createsSeventhJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	// Vector3f expected = new Vector3f(1f, 1f, 0f), result =
	// body.getJointLocs()[6];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	// }
	//
	// @Test
	// public void createsEighthJoint() throws Exception {
	// new Expectations() {
	// {
	// physics.getDynamicsWorld();
	// result = world;
	// minTimes = 1;
	// }
	// };
	// RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
	//
	// Vector3f expected = new Vector3f(2f, 1f, 0f), result =
	// body.getJointLocs()[7];
	//
	// assertTrue(String.format("Expected %s, got %s", expected, result),
	// expected.epsilonEquals(result, 0.0001f));
	// }

}
