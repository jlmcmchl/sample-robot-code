package net.teamrush27.frc2019.util.math;

import java.text.DecimalFormat;
import net.teamrush27.frc2018.util.interpolate.Interpolable;


public class Translation2d implements Interpolable<Translation2d> {
    protected static final Translation2d kIdentity = new Translation2d();

    public static final Translation2d identity() {
        return kIdentity;
    }

    protected double x;
    protected double y;

    public Translation2d() {
        x = 0;
        y = 0;
    }

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Translation2d(Translation2d other) {
        x = other.x;
        y = other.y;
    }

    public Translation2d(Translation2d start, Translation2d end) {
        x = end.x - start.x;
        y = end.y - start.y;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     * 
     * @return sqrt(x^2 + y^2)
     */
    public double norm() {
        return Math.hypot(x, y);
    }

    public double norm2() {
        return x * x + y * y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
    	this.y = y;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     * 
     * @param other
     *            The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation2d translateBy(Translation2d other) {
        return new Translation2d(x + other.x, y + other.y);
    }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * 
     * @param rotation
     *            The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2d rotateBy(Rotation2d rotation) {
        return new Translation2d(x * rotation.cos() - y * rotation.sin(), x * rotation.sin() + y * rotation.cos());
    }

    public Rotation2d direction() {
        return new Rotation2d(x, y, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     * 
     * @return Translation by -x and -y.
     */
    public Translation2d inverse() {
        return new Translation2d(-x, -y);
    }

    @Override
    public Translation2d interpolate(Translation2d other, double x) {
        if (x <= 0) {
            return new Translation2d(this);
        } else if (x >= 1) {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    public Translation2d extrapolate(Translation2d other, double x) {
        return new Translation2d(x * (other.x - this.x) + this.x, x * (other.y - y) + y);
    }

    public Translation2d scale(double s) {
        return new Translation2d(x * s, y * s);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x) + "," + fmt.format(y) + ")";
    }

    public static double dot(Translation2d a, Translation2d b) {
        return a.x * b.x + a.y * b.y;
    }

    public static Rotation2d getAngle(Translation2d a, Translation2d b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation2d();
        }
        return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    public static double cross(Translation2d a, Translation2d b) {
        return a.x * b.y - a.y * b.x;
    }
}
