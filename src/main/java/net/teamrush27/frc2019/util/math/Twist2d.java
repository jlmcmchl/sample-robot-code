package net.teamrush27.frc2019.util.math;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * 
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class Twist2d {
    protected static final Twist2d IDENTITY = new Twist2d(0.0, 0.0, 0.0);

    public static final Twist2d identity() {
        return IDENTITY;
    }

    public final double deltaX;
    public final double deltaY;
    public final double deltaTheta; // Radians!

    public Twist2d(double deltaX, double deltaY, double deltaTheta) {
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.deltaTheta = deltaTheta;
    }

    public Twist2d scaled(double scale) {
        return new Twist2d(deltaX * scale, deltaY * scale, deltaTheta * scale);
    }
    
    @Override
	public String toString(){
    	return String.format("X: %s Y: %s theta: %s",deltaX,deltaY,deltaTheta);
	}
}
