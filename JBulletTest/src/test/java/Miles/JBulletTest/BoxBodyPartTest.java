package Miles.JBulletTest;

import static org.junit.Assert.*;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.BoxBodyPart;

import org.junit.Test;

import junit.framework.TestCase;

public class BoxBodyPartTest {

	@Test
	public void oneAPperSideCreatesCorrectPoints() throws Exception {
		BoxBodyPart bbp = new BoxBodyPart(new Vector3f(0, 1, 0), new Vector3f(
				2, 1, 4),1);

		Vector3f expected = new Vector3f(0, 1, -2), result = bbp.getNextAP()
				.getPoint();

		assertTrue(String.format("West point: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(1, 1, 0);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("North point: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(0, 1, 2);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("East point: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(-1, 1, 0);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("East point: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void twoAPsperSideCreateCorrectPoints() throws Exception {
		BoxBodyPart bbp = new BoxBodyPart(new Vector3f(0, 1, 0), new Vector3f(
				3, 1, 6),2);

		Vector3f expected = new Vector3f(-0.5f, 1, -3), result = bbp
				.getNextAP().getPoint();

		assertTrue(String.format("West point 1: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(0.5f, 1, -3);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("West point 2: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
		
		expected = new Vector3f(1.5f, 1, -1);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("North point 1: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));

		expected = new Vector3f(1.5f, 1, 1);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("North point 2: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
		
		expected = new Vector3f(0.5f, 1, 3);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("East point 1: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
		
		expected = new Vector3f(-0.5f, 1, 3);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("East point 2: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
		
		expected = new Vector3f(-1.5f, 1, 1);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("East point 2: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
		
		expected = new Vector3f(-1.5f, 1, -1);
		result = bbp.getNextAP().getPoint();

		assertTrue(String.format("East point 2: expected %s, got %s", expected,
				result), expected.epsilonEquals(result, 0.0001f));
	}

}
