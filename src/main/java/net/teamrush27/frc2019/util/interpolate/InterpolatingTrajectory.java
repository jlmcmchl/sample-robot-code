package net.teamrush27.frc2019.util.interpolate;

//import jaci.pathfinder.Trajectory;

/*public class InterpolatingTrajectory extends Trajectory {

    InterpolatingSegment[] segments;

    public InterpolatingTrajectory(Segment[] segments) {
        super(segments);
    }

    public InterpolatingTrajectory(int length) {
        super(length);
    }

    public InterpolatingTrajectory(Trajectory trajectory) {
        super(trajectory.length());
        this.segments = new InterpolatingSegment[trajectory.length()];

        for (int i = 0; i < trajectory.length(); i++) {
            this.segments[i] = new InterpolatingSegment(trajectory.get(i));
        }
    }

    public enum Field {
        DT,
        X,
        Y,
        POSITION,
        VELOCITY,
        ACCELERATION,
        JERK,
        HEADING
    }

    @Override
    public InterpolatingSegment get(int index) {
        return (InterpolatingSegment)segments[index];
    }

    public InterpolatingSegment getSegmentByFieldBetween(Field field, int low, int high, double value) {
        InterpolatingSegment segLow = this.get(low);
        InterpolatingSegment segHigh = this.get(high); // heh
        double part = value;
        double whole = 1;

        switch(field) {
            case DT:
                part -= low;
                whole = high - low;
                break;
            case X:
                part -= segLow.x;
                whole = segHigh.x - segLow.x;
                break;
            case Y:
                part -= segLow.y;
                whole = segHigh.y - segLow.y;
                break;
            case POSITION:
                part -= segLow.position;
                whole = segHigh.position - segLow.position;
                break;
            case VELOCITY:
                part -= segLow.velocity;
                whole = segHigh.velocity - segLow.velocity;
                break;
            case ACCELERATION:
                part -= segLow.acceleration;
                whole = segHigh.acceleration - segLow.acceleration;
                break;
            case JERK:
                part -= segLow.jerk;
                whole = segHigh.jerk - segLow.jerk;
                break;
            case HEADING:
                part -= segLow.heading;
                whole = segHigh.heading - segLow.heading;
                break;
        }

        return segLow.interpolate(segHigh, part / whole);
    }
}
*/