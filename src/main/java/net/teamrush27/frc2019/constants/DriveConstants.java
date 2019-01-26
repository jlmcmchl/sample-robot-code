package net.teamrush27.frc2019.constants;

public class DriveConstants {
	public static double PID_P = 5; //1.0;
    public static double PID_I = 0;// 0.002;
    public static double PID_D = 100; // 100.0;
    public static double PID_F = .2; // .45;
    public static int PID_I_ZONE = 700;
    public static double PID_RAMP_RATE = .125; // V/s
    

    
    public static double NOMINAL_OUTPUT = 0.5; // V
    
    // 13 f/s in in/s
    public static double MAX_VELOCITY = 13.0 * 12.0;
    
    // 5 f/s/s in in/s/s
    public static double MAX_ACCELERATION = 5.0 * 12.0;
    
    public static int MAX_CONTINUOUS_CURRENT = 18;
    
    public static int MAX_PEAK_CURRENT = 37;
    public static int PEAK_CURRENT_DURATION = 20;
}
