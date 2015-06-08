package me.mb.resmach.physsim;

import static org.junit.Assert.*;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.PhysicsUtils;

import org.junit.Test;

public class CylinderBodyPartTest {

	@Test
	public void xCylinderHasCorrectAPs() {
		CylinderBodyPart cbp = new CylinderBodyPart(new Vector3f(0, 0, 0),
				PhysicsUtils.Axis.X, 1f, 2f);

		Vector3f expected = new Vector3f(-1, 0, 0), result = cbp.getNextAP()
				.getPoint();

		assertTrue(String.format("Point 1: expected %s but got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(1, 0, 0);
		result = cbp.getNextAP().getPoint();

		assertTrue(String.format("Point 2: expected %s but got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
	}
	@Test
	public void yCylinderHasCorrectAPs() {
		CylinderBodyPart cbp = new CylinderBodyPart(new Vector3f(0, 0, 0),
				PhysicsUtils.Axis.Y, 1f, 2f);

		Vector3f expected = new Vector3f(0,-1, 0), result = cbp.getNextAP()
				.getPoint();

		assertTrue(String.format("Point 1: expected %s but got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(0,1, 0);
		result = cbp.getNextAP().getPoint();

		assertTrue(String.format("Point 2: expected %s but got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
	}
	
	@Test
	public void zCylinderHasCorrectAPs() {
		CylinderBodyPart cbp = new CylinderBodyPart(new Vector3f(0, 0, 0),
				PhysicsUtils.Axis.Z, 1f, 2f);

		Vector3f expected = new Vector3f(0,0,-1), result = cbp.getNextAP()
				.getPoint();

		assertTrue(String.format("Point 1: expected %s but got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(0,0,1);
		result = cbp.getNextAP().getPoint();

		assertTrue(String.format("Point 2: expected %s but got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
	}
}
