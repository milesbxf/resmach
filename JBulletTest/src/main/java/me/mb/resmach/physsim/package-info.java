/**
Generates a 3D robot from a body plan and simulates the given motor commands, with sensory feedback returned.

It uses the Bullet Physics coordinate system, with y+ pointing upwards and the xz plane is horizontal 
(with x+ pointing up and z+ pointing right). 

Rotations in any given plane are left-hand, or clockwise, so e.g. transforming from polar to Cartesian 
is performed with, e.g. (x,z) = (cos(a),-sin(a)). 
 */
package me.mb.resmach.physsim;