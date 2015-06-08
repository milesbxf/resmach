package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

import me.mb.resmach.physsim.util.PhysicsUtils;

public class BoxBodyPart extends BodyPart {
	
	private final Vector3f size;
	
	
	public BoxBodyPart(Vector3f position, Vector3f size, int nAPsPerSide) {
		super(position,PhysicsUtils.createBox(position.x, position.y, position.z,
				size.x, size.y, size.z));
		this.size = size;
		calculateAPs(nAPsPerSide);
	}

	private void calculateAPs(int nPointsPerSide) {
		Vector3f bottomLeftCorner = new Vector3f(position.x - size.x / 2,
				position.y, position.z - size.z / 2);

		for (int point = 0; point < nPointsPerSide; point++) {
			Vector3f newVec = new Vector3f(bottomLeftCorner);
			newVec.add(new Vector3f(
					(point + 1) * size.x / (nPointsPerSide + 1), 0, 0));
			if (nextAP == null)
				nextAP = new AttachmentPoint(newVec);
			else {
				AttachmentPoint tempAP = nextAP;
				while (tempAP.getNext() != null) {
					tempAP = tempAP.getNext();
				}
				tempAP.setNext(new AttachmentPoint(newVec));
			}
		}

		Vector3f topLeftCorner = new Vector3f(position.x + size.x / 2,
				position.y, position.z - size.z / 2);

		for (int point = 0; point < nPointsPerSide; point++) {
			Vector3f newVec = new Vector3f(topLeftCorner);
			newVec.add(new Vector3f(
					0,0,(point + 1) * size.z / (nPointsPerSide + 1)));
			if (nextAP == null)
				nextAP = new AttachmentPoint(newVec);
			else {
				AttachmentPoint tempAP = nextAP;
				while (tempAP.getNext() != null) {
					tempAP = tempAP.getNext();
				}
				tempAP.setNext(new AttachmentPoint(newVec));
			}
		}

		Vector3f topRightCorner = new Vector3f(position.x + size.x / 2,
				position.y, position.z + size.z / 2);

		for (int point = 0; point < nPointsPerSide; point++) {
			Vector3f newVec = new Vector3f(topRightCorner);
			newVec.add(new Vector3f(
					(point + 1) * -size.x / (nPointsPerSide + 1),0,0));
			
			if (nextAP == null)
				nextAP = new AttachmentPoint(newVec);
			else {
				AttachmentPoint tempAP = nextAP;
				while (tempAP.getNext() != null) {
					tempAP = tempAP.getNext();
				}
				tempAP.setNext(new AttachmentPoint(newVec));
			}
		}

		Vector3f bottomRightCorner = new Vector3f(position.x - size.x / 2,
				position.y, position.z + size.z / 2);

		for (int point = 0; point < nPointsPerSide; point++) {
			Vector3f newVec = new Vector3f(bottomRightCorner);
			newVec.add(new Vector3f(
					0,0,(point + 1) * -size.z / (nPointsPerSide + 1)));
			if (nextAP == null)
				nextAP = new AttachmentPoint(newVec);
			else {
				AttachmentPoint tempAP = nextAP;
				while (tempAP.getNext() != null) {
					tempAP = tempAP.getNext();
				}
				tempAP.setNext(new AttachmentPoint(newVec));
			}
		}
	}

}
