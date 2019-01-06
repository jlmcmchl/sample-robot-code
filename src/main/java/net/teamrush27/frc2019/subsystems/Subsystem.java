package net.teamrush27.frc2019.subsystems;

import net.teamrush27.frc2019.loops.Looper;

public abstract class Subsystem {

    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);
    
    public abstract void test();
	
}
