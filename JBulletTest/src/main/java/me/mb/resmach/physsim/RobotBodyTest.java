package me.mb.resmach.physsim;

import static org.junit.Assert.*;

import java.util.List;

import javax.vecmath.Vector3f;

import mockit.Expectations;
import mockit.Mocked;

import org.junit.Test;

import com.bulletphysics.dynamics.DynamicsWorld;
import com.google.common.collect.Lists;

public class RobotBodyTest {

	@Mocked
	Physics physics;
	@Mocked
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
	public void createsFirstJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(0f, 1f, -1f), result = body.jointLocs[0];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsSecondJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(0f, 1f, -2f), result = body.jointLocs[1];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsThirdJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);


		Vector3f expected = new Vector3f(-1.5f, 1f, 0f), result = body.bodyLocs[2];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));

	}

	@Test
	public void createsFourthJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(-2f, 1f, 0f), result = body.jointLocs[3];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsFifthJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(0f, 1f, 1f), result = body.jointLocs[4];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsSixthJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(0f, 1f, 2f), result = body.jointLocs[5];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsSeventhJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(1f, 1f, 0f), result = body.jointLocs[6];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void createsEighthJoint() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);

		Vector3f expected = new Vector3f(2f, 1f, 0f), result = body.jointLocs[7];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	
	@Test
	public void createsFirstBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(0f, 1f, -1.5f), result = body.bodyLocs[0];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	

	@Test
	public void createsSecondBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(0f, 1f, -2.5f), result = body.bodyLocs[1];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	

	@Test
	public void createsThirdBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(-1.5f, 1f, 0f), result = body.bodyLocs[2];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	
	@Test
	public void createsFourthBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(-2.5f, 1f, 0f), result = body.bodyLocs[3];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	

	@Test
	public void createsFifthBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(0f, 1f, 1.5f), result = body.bodyLocs[4];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	

	@Test
	public void createsSixthBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(0f, 1f, 2.5f), result = body.bodyLocs[5];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	


	@Test
	public void createsSeventhBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(1.5f, 1f, 0f), result = body.bodyLocs[6];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
	

	@Test
	public void createsEighthBody() throws Exception {
		new Expectations() {
			{
				physics.getDynamicsWorld();
				result = world;
				minTimes = 1;
			}
		};
		RobotBody body = new RobotBody(physics, attachmentLocs, perimeterLocs);
		
		Vector3f expected = new Vector3f(2.5f, 1f, 0f), result = body.bodyLocs[7];

		assertTrue(String.format("Expected %s, got %s", expected, result),
				expected.epsilonEquals(result, 0.0001f));
	}
}
