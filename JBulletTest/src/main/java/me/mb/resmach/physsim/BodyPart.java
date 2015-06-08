package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import com.bulletphysics.dynamics.RigidBody;

public abstract class BodyPart {

	protected Vector3f position;
	protected final RigidBody body;
	protected AttachmentPoint nextAP;
	
	public BodyPart(Vector3f position,RigidBody body) {
		this.position = position;
		this.body = body;
	}
	
	public BodyPart(RigidBody body) {
		this.body = body;
	}

	public AttachmentPoint getNextAP() {
		if (nextAP == null)
			return null;
		AttachmentPoint apToReturn = nextAP;
		nextAP = nextAP.getNext();
		return apToReturn;
	}
	
}
