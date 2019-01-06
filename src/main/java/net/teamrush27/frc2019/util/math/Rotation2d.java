package net.teamrush27.frc2019.util.math;

import java.text.DecimalFormat;
import net.teamrush27.frc2019.util.interpolate.Interpolable;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 * 
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Rotation2d implements Interpolable<Rotation2d> {
    protected static final Rotation2d IDENTITY = new Rotation2d();

    public static final Rotation2d identity() {
        return IDENTITY;
    }

    protected static final double kEpsilon = 1E-9;

    protected double cosAngle;
    protected double sinAngle;

    public Rotation2d() {
        this(1, 0, false);
    }

    public Rotation2d(double x, double y, boolean normalize) {
        cosAngle = x;
        sinAngle = y;
        if (normalize) {
            normalize();
        }
    }

    public Rotation2d(Rotation2d other) {
        cosAngle = other.cosAngle;
        sinAngle = other.sinAngle;
    }

    public Rotation2d(Translation2d direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    public static Rotation2d fromRadians(double angleRadians) {
        return new Rotation2d(Math.cos(angleRadians), Math.sin(angleRadians), false);
    }

    public static Rotation2d fromDegrees(double angleDegrees) {
        return fromRadians(Math.toRadians(angleDegrees));
    }

    /**
     * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
     * Normalizing forces us to re-scale the sin and cos to reset rounding errors.
     */
    public void normalize() {
        double magnitude = Math.hypot(cosAngle, sinAngle);
        if (magnitude > kEpsilon) {
            sinAngle /= magnitude;
            cosAngle /= magnitude;
        } else {
            sinAngle = 0;
            cosAngle = 1;
        }
    }

    public double cos() {
        return cosAngle;
    }

    public double sin() {
        return sinAngle;
    }

    public double tan() {
        if (Math.abs(cosAngle) < kEpsilon) {
            if (sinAngle >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sinAngle / cosAngle;
    }

    public double getRadians() {
        return Math.atan2(sinAngle, cosAngle);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     * 
     * @param other
     *            The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(cosAngle * other.cosAngle - sinAngle * other.sinAngle,
                cosAngle * other.sinAngle + sinAngle * other.cosAngle, true);
    }

    public Rotation2d normal() {
        return new Rotation2d(-sinAngle, cosAngle, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     * 
     * @return The opposite of this rotation.
     */
    public Rotation2d inverse() {
        return new Rotation2d(cosAngle, -sinAngle, false);
    }

    public boolean isParallel(Rotation2d other) {
        return MathUtils.epsilonEquals(Translation2d.cross(toTranslation(), other.toTranslation()), 0.0, kEpsilon);
    }

    public Translation2d toTranslation() {
        return new Translation2d(cosAngle, sinAngle);
    }

    @Override
    public Rotation2d interpolate(Rotation2d other, double x) {
        if (x <= 0) {
            return new Rotation2d(this);
        } else if (x >= 1) {
            return new Rotation2d(other);
        }
        double angle_diff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(getDegrees()) + " deg)";
    }
}
