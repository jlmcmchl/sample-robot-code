package net.teamrush27.frc2019.constants;

public class RobotConstants {

    // Looping Parameters
    // Minimum time between calling onLoop() for SubsystemManager
    // Will be longer if onLoop() takes longer
    public static final double LOOPER_DELTA_TIME = 0.02; // 50hz

    // Talon Parameters
    public static final int TALON_CONFIG_TIMEOUT = 100; // 100ms

    // Drivetrain Characteristics
    // Scrub Factor:
    //   Ratio between read encoder ticks 
    //      and actual travelled distance when turning
    public static final double SCRUB_FACTOR = 1.0;

    // c-c Track Width
    // Distance between left and right side of Drivetrain
    public static final int TRACK_WIDTH = 28; // 28in

    // How many encoder ticks are measured per inch travelled
    // 4096 tick encoder on a 4in diameter wheel
    public static final double DT_TICKS_PER_INCH = 4096.0 / (4 * Math.PI);

    // Motion Magic configured maximum velocity
    // Units: Encoder ticks per 100ms
    public static final int DRIVE_CRUISE_VELOCITY = 50; // theoretical 3900;

    // Motion Magic configured maximum acceleration
    // Units: Encoder ticks per 100ms per 1s
    public static final int DRIVE_ACCELERATION = 100; // theoretical 8000;
}
