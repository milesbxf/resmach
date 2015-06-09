package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.PhysicsUtils;

import com.bulletphysics.dynamics.RigidBody;

public class BoxBodyPart extends BodyPart {

	public BoxBodyPart(Vector3f position,Vector3f size) {
		super(position, PhysicsUtils.createBox(position, size));
	}

}
