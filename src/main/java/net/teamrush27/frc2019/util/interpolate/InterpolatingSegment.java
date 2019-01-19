package net.teamrush27.frc2019.util.interpolate;

//import jaci.pathfinder.Trajectory.Segment;

/*public class InterpolatingSegment extends Segment implements Interpolable<InterpolatingSegment> {
    public InterpolatingSegment(double dt, double x, double y, double position, double velocity, double acceleration, double jerk, double heading) {
        super(dt, x, y, position, velocity, acceleration, jerk, heading);
    }

    public InterpolatingSegment(Segment segment) {
        super(segment.dt, segment.x, segment.y, segment.position, segment.velocity, segment.acceleration, segment.jerk, segment.heading);
    }

    @Override
    public InterpolatingSegment interpolate(InterpolatingSegment other, double weight) {
        if (x == 0) {
            return this;
        } else if (x == 1) {
            return other;
        }

        double dt = other.dt * weight + this.dt * (1 - weight);
        double x = other.x * weight + this.x * (1 - weight);
        double y = other.y * weight + this.y * (1 - weight);
        double pos = other.position * weight + this.position * (1 - weight);
        double vel = other.velocity * weight + this.velocity * (1 - weight);
        double acc = other.acceleration * weight + this.acceleration * (1 - weight);
        double jerk = other.jerk * weight + this.jerk * (1 - weight);
        double head = other.heading * weight + this.heading * (1 - weight);

        return new InterpolatingSegment(dt, x, y, pos, vel, acc, jerk, head);
    }
}
*/