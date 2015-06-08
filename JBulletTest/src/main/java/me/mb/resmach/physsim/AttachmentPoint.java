package me.mb.resmach.physsim;

import javax.vecmath.Vector3f;

public class AttachmentPoint {
	private final Vector3f point;
	private AttachmentPoint next;
	private AttachmentPoint prev;
	private AttachmentPoint parent;
	private AttachmentPoint child;

	public AttachmentPoint(Vector3f point) {
		this.point = point;
	}
	
	public AttachmentPoint getNext() {
		return next;
	}
	public void setNext(AttachmentPoint next) {
		next.prev = this;
		this.next = next;
	}
	public AttachmentPoint getPrev() {
		return prev;
	}
	public void setPrev(AttachmentPoint prev) {
		prev.next = this;
		this.prev = prev;
	}
	public AttachmentPoint getParent() {
		return parent;
	}
	public void setParent(AttachmentPoint parent) {
		parent.child = this;
		this.parent = parent;
	}
	public AttachmentPoint getChild() {
		return child;
	}
	public void setChild(AttachmentPoint child) {
		child.parent = this;
		this.child = child;
	}
	public Vector3f getPoint() {
		return point;
	}
	public boolean hasChild() {
		return child != null;
	}
}
