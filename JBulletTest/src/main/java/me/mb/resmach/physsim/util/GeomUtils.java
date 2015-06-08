package me.mb.resmach.physsim.util;

import javax.vecmath.Vector2f;

public class GeomUtils {
	public static final float PIf = (float)Math.PI;
	
	public static float degToRad(float degrees) {
		return (degrees * PIf / 180);
	}
	
	public static float tan(float x) {
		return (float)Math.tan(x);
	}

	public static Vector2f getPointOnRectPerimeter(float xsize, float ysize,
			float angle) {
		//Move angle so its in the range [-PI,PI]
		while(angle < -PIf) {
			angle += 2*PIf;
		}
		while(angle > PIf) {
			angle -= 2*PIf;
		}
		//use atan2 to calculate quadrant
		float rectATan2 = (float) Math.atan2(ysize,xsize),
				tana = tan(angle);
		
		if(Math.abs(tana) > 1e5)
			tana = 0;

		float x = 0,y = 0;
		if((-rectATan2 < angle) && (angle <= rectATan2)) { //Quadrant 1
			x = xsize/2; y = -(xsize/2)*tana;
		} else if((rectATan2 < angle) && (angle <= (PIf - rectATan2))) { //Quadrant 2
			x = (ysize/2) * tana; y = -(ysize/2);
		} else if(((PIf - rectATan2) < angle) || (angle <= -(PIf - rectATan2))) { //Quadrant 3
			x = -(xsize/2); y = (xsize/2)*tana;
		} else if(((PIf + rectATan2) < angle) || (angle <= - rectATan2)) { //Quadrant 4
			x = -(ysize/2) * tana; y = (ysize/2);	
		} else { //shouldn't happen
			throw new IllegalArgumentException("Angle is outside range");
		}
		return new Vector2f(x, y);
	}
}
