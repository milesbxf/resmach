package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.PhysicsUtils;
import me.mb.resmach.physsim.util.PhysicsUtils.Axis;

import com.bulletphysics.dynamics.RigidBody;

public class CylinderBodyPart extends BodyPart {

	private final float radius,length;
	
	public CylinderBodyPart(Vector3f position, float orientation, float radius, float length) {
		super(position, PhysicsUtils.createCylinder(PhysicsUtils.Axis.Y, position.x, position.y, position.z, orientation,radius, length));
		this.radius=radius;this.length=length;
	}	
	
	
}
