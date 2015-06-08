package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.PhysicsUtils;
import me.mb.resmach.physsim.util.PhysicsUtils.Axis;

import com.bulletphysics.dynamics.RigidBody;

public class CylinderBodyPart extends BodyPart {

	private final PhysicsUtils.Axis axis;
	private final float radius,length;
	
	public CylinderBodyPart(Vector3f position, PhysicsUtils.Axis x, float radius, float length) {
		super(position, PhysicsUtils.createCylinder(x, position.x, position.y, position.z, radius, length));
		this.axis=x; this.radius=radius;this.length=length;
		calculateAPs();
	}
	
	
	private void calculateAPs() {
		Vector3f pt = new Vector3f(position),
				pt2 = new Vector3f(position);
		if(axis == Axis.X) {
			pt.add(new Vector3f(-length/2,0,0));
			pt2.add(new Vector3f(length/2,0,0));			
		} else if(axis == Axis.Y) {
			pt.add(new Vector3f(0,-length/2,0));
			pt2.add(new Vector3f(0,length/2,0));			
		} else if(axis == Axis.Z) {
			pt.add(new Vector3f(0,0,-length/2));
			pt2.add(new Vector3f(0,0,length/2));			
		} else {
			throw new IllegalArgumentException("Axis must be X,Y or Z");
		}
		nextAP = new AttachmentPoint(pt);
		nextAP.setNext(new AttachmentPoint(pt2));
	}
	
}
