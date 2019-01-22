package net.teamrush27.frc2019.constants;

public class ChezyConstants {
    // DRIVE CONSTANTS
    public static double PID_P = .251; //1.0;
    public static double PID_I = 0;// 0.002;
    public static double PID_D = 100.0; // 100.0;
    public static double PID_F = .4; // .45;
    public static int PID_I_ZONE = 700;
    public static double PID_RAMP_RATE = .125; // V/s
    
    
    public static double ROTATE_PID_P = 1.0; //1.0;
    public static double ROTATE_PID_I = 0.002;// 0.002;
    public static double ROTATE_PID_D = 100.0; // 100.0;
    public static double ROTATE_PID_F = .65; // .45;
    public static int ROTATE_PID_I_ZONE = 700;
    
    
    public static double NOMINAL_OUTPUT = 0.5; // V

    // 13 f/s in in/s
    public static double MAX_VELOCITY = 13.0 * 12.0;

    // 5 f/s/s in in/s/s
    public static double MAX_ACCELERATION = 5.0 * 12.0;
    
    // FOLLOWING CONSTANTS
    public static double SEGMENT_COMPLETION_TOLERANCE = 0.1; // inches

    public static final double FOLLOWING_MAX_ACCELERATION = 120.0; // inches per second ^2
    public static double MAX_SPEED = 120.0; // inches per second
    public static double P = .01;
    public static double I = 0;
    public static double D = 20;
    public static double V = .08;
    public static double A = 0;
    public static double FEED_FORWARD_VELOCITY = 1;
    public static double FEED_FORWARD_ACCELERATION = 0.05;
    public static double GOAL_POSITION_TOLERANCE = 0.75;
    public static double GOAL_VELOCITY_TOLERANCE = 12.0;
    public static double STOP_STEERING_DISTANCE = 5.0;


    public static double MIN_LOOKAHEAD_DISTANCE = 12.0; // inches
    public static double MIN_LOOKAHEAD_SPEED = 9.0; // inches per second
    public static double MAX_LOOKAHEAD_DISTANCE = 24.0; // inches
    public static double MAX_LOOKAHEAD_SPEED = 120.0; // inches per second
    public static double LOOKAHEAD_DISTANCE_WINDOW = MAX_LOOKAHEAD_DISTANCE - MIN_LOOKAHEAD_DISTANCE;
    public static double LOOKAHEAD_SPEED_WINDOW = MAX_LOOKAHEAD_SPEED - MIN_LOOKAHEAD_SPEED;

    public static double INTERIAL_STEERING_GAIN = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec

    // ROBOT CONSTANTS
    // Wheel Base
    public static double DRIVE_WHEEL_DIAMETER = 7.75; // in inches
    public static double TRACK_WIDTH = 25; // in inches
    public static double SCRUB_FACTOR = 1.25; // 2

    // Extremities
    // below in inches
    public static double CENTER_TO_FRONT_BUMPER = 14.5;
    public static double CENTER_TO_EXTENDED_HARVESTER = 23.11;
    public static double CENTER_TO_REAR_BUMPER = 14.5;
    public static double CENTER_TO_SIDE_BUMPER = 19.25;


    // Camera Position
    public static double CAMERA_X_OFFSET = 0.0;
    public static double CAMERA_Y_OFFSET = 0.0;
    public static double CAMERA_Z_OFFSET = 0.0;
    public static double CAMERA_PITCH_ANGLE = 0.0; // Measured on 4/26
    public static double CAMERA_YAW_ANGLE = 0.0;
    public static double CAMERA_DEADBAND = 0.0;

    // Looping Parameters
    public static double LOOPER_DELTA_TIME = 0.005; // 200hz

    // Talon Parameters
    public static int TALON_CONFIG_TIMEOUT = 0; // 100ms

    // Drive Motion Constants
    public static double kDriveKv = 0.135;
    public static double kDriveWheelDiameterInches = 7.75;
    public static double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2;
    public static double kRobotLinearInertia = 60.0;
    public static double kDriveKa = 0.012;
    public static double kDriveVIntercept = 1.055;
    public static double kRobotAngularInertia = 10.0;
    public static double kRobotAngularDrag = 12.0;
    public static double kDriveWheelTrackWidthInches = 25.0;
    public static double kTrackScrubFactor = 1.25;
    public static double kPathLookaheadTime = 0.4;
    public static double kPathMinLookaheadDistance = 24.0;
    public static double kPathKX = 4.0;

}
