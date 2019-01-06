package net.teamrush27.frc2019.util.math;

import net.teamrush27.frc2019.util.interpolate.Interpolable;

/**
 * Represents a 2d pose (rigid transform) containing translational and rotational elements.
 * 
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 * 
 * @author team254
 */
public class RigidTransform2d implements Interpolable<RigidTransform2d> {
    protected static final double MAX_ERROR = 1E-9;

    protected static final RigidTransform2d IDENTITY = new RigidTransform2d();

    public static final RigidTransform2d identity() {
        return IDENTITY;
    }

    protected Translation2d translation;
    protected Rotation2d rotation;

    public RigidTransform2d() {
        translation = new Translation2d();
        rotation = new Rotation2d();
    }

    public RigidTransform2d(Translation2d translation, Rotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public RigidTransform2d(RigidTransform2d other) {
        translation = new Translation2d(other.translation);
        rotation = new Rotation2d(other.rotation);
    }

    public static RigidTransform2d fromTranslation(Translation2d translation) {
        return new RigidTransform2d(translation, new Rotation2d());
    }

    public static RigidTransform2d fromRotation(Rotation2d rotation) {
        return new RigidTransform2d(new Translation2d(), rotation);
    }

    /**
     * Obtain a new RigidTransform2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static RigidTransform2d exp(Twist2d twist) {
        double sinTheta = Math.sin(twist.deltaTheta);
        double cosTheta = Math.cos(twist.deltaTheta);
        double sinThetaByTheta, oneMinusCosThetaByTheta;
        if (Math.abs(twist.deltaTheta) < MAX_ERROR) {
            sinThetaByTheta = 1.0 - 1.0 / 6.0 * twist.deltaTheta * twist.deltaTheta;
            oneMinusCosThetaByTheta = .5 * twist.deltaTheta;
        } else {
            sinThetaByTheta = sinTheta / twist.deltaTheta;
            oneMinusCosThetaByTheta = (1.0 - cosTheta) / twist.deltaTheta;
        }
        return new RigidTransform2d(
        		new Translation2d(
        				twist.deltaX * sinThetaByTheta - twist.deltaY * oneMinusCosThetaByTheta, 
        				twist.deltaX * oneMinusCosThetaByTheta + twist.deltaY * sinThetaByTheta),
                new Rotation2d(cosTheta, sinTheta, false));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist2d log(RigidTransform2d transform) {
        final double deltaTheta = transform.getRotation().getRadians();
        final double halfDeltaTheta = 0.5 * deltaTheta;
        final double cosMinusOne = transform.getRotation().cos() - 1.0;
        double halfThetaByTanOfHalfDeltaTheta;
        if (Math.abs(cosMinusOne) < MAX_ERROR) {
            halfThetaByTanOfHalfDeltaTheta = 1.0 - 1.0 / 12.0 * deltaTheta * deltaTheta;
        } else {
            halfThetaByTanOfHalfDeltaTheta = -(halfDeltaTheta * transform.getRotation().sin()) / cosMinusOne;
        }
        final Translation2d translation = transform.getTranslation()
                .rotateBy(new Rotation2d(halfThetaByTanOfHalfDeltaTheta, -halfDeltaTheta, false));
        return new Twist2d(translation.x(), translation.y(), deltaTheta);
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public void setTranslation(Translation2d translation) {
        this.translation = translation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public void setRotation(Rotation2d rotation) {
        this.rotation = rotation;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     * 
     * @param other
     *            The other transform.
     * @return This transform * other
     */
    public RigidTransform2d transformBy(RigidTransform2d other) {
        return new RigidTransform2d(translation.translateBy(other.translation.rotateBy(rotation)),
                rotation.rotateBy(other.rotation));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     * 
     * @return The opposite of this transform.
     */
    public RigidTransform2d inverse() {
        Rotation2d rotationInverted = rotation.inverse();
        return new RigidTransform2d(translation.inverse().rotateBy(rotationInverted), rotationInverted);
    }

    public RigidTransform2d normal() {
        return new RigidTransform2d(translation, rotation.normal());
    }

    /**
     * Finds the point where the heading of this transform intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation2d intersection(RigidTransform2d other) {
        final Rotation2d otherRotation = other.getRotation();
        if (rotation.isParallel(otherRotation)) {
            // Lines are parallel.
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation.cos()) < Math.abs(otherRotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if the heading of this transform is colinear with the heading of another.
     */
    public boolean isColinear(RigidTransform2d other) {
        final Twist2d twist = log(inverse().transformBy(other));
        return (MathUtils.epsilonEquals(twist.deltaY, 0.0, MAX_ERROR) && MathUtils.epsilonEquals(twist.deltaTheta, 0.0, MAX_ERROR));
    }

    private static Translation2d intersectionInternal(RigidTransform2d a, RigidTransform2d b) {
        final Rotation2d aRotation = a.getRotation();
        final Rotation2d bRotation = b.getRotation();
        final Translation2d aTranslation = a.getTranslation();
        final Translation2d bTranslation = b.getTranslation();

        final double tanB = bRotation.tan();
        final double t = ((aTranslation.x() - bTranslation.x()) * tanB + bTranslation.y() - aTranslation.y())
                / (aRotation.sin() - aRotation.cos() * tanB);
        return aTranslation.translateBy(aRotation.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this transform assuming constant curvature.
     */
    @Override
    public RigidTransform2d interpolate(RigidTransform2d other, double x) {
        if (x <= 0) {
            return new RigidTransform2d(this);
        } else if (x >= 1) {
            return new RigidTransform2d(other);
        }
        final Twist2d twist = RigidTransform2d.log(inverse().transformBy(other));
        return transformBy(RigidTransform2d.exp(twist.scaled(x)));
    }

    @Override
    public String toString() {
        return "T:" + translation.toString() + ", R:" + rotation.toString();
    }
}
