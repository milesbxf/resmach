package Miles.JBulletTest;

import static org.junit.Assert.*;

import javax.vecmath.Vector2f;

import me.mb.resmach.physsim.util.GeomUtils;

import org.junit.Test;

public class GeomUtilsTest {

	@Test
	public void testGetPointOnRectPerimeterRegionE() {
		Vector2f expected = new Vector2f(2, 0);
		Vector2f result = GeomUtils.getPointOnRectPerimeter(4, 2,
				GeomUtils.degToRad(0));
		assertTrue(String.format("0 degrees case: expected %s got %s",
				expected, result), expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void testGetPointOnRectPerimeterRegionS() throws Exception {
		Vector2f expected = new Vector2f((float) Math.sqrt(3), -1);
		Vector2f result = GeomUtils.getPointOnRectPerimeter(4, 2,
				GeomUtils.degToRad(60));
		assertTrue(String.format("60 degrees case: expected %s got %s",
				expected, result), expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void testGetPointOnRectPerimeterLeftDiag() throws Exception {
		Vector2f expected = new Vector2f(-2, 2 * GeomUtils.tan(GeomUtils
				.degToRad(6)));
		Vector2f result = GeomUtils.getPointOnRectPerimeter(4, 2,
				GeomUtils.degToRad(186));
		assertTrue(String.format("186 degrees case: expected %s got %s",
				expected, result), expected.epsilonEquals(result, 0.0001f));
	}

	@Test
	public void testGetPointOnRectPerimeterRegionN() throws Exception {
		Vector2f expected = new Vector2f(
				-GeomUtils.tan(GeomUtils.degToRad(300)), 1);
		Vector2f result = GeomUtils.getPointOnRectPerimeter(4, 2,
				GeomUtils.degToRad(300));
		assertTrue(String.format("300 degrees case: expected %s got %s",
				expected, result), expected.epsilonEquals(result, 0.0001f));
	}
}
