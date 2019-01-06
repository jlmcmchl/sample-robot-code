package net.teamrush27.frc2019.subsystems;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import net.teamrush27.frc2019.loops.Looper;

public class SubsystemManager {

	private final Set<Subsystem> subsystems = new HashSet<Subsystem>();
	
	public SubsystemManager(Subsystem ...subsystems){
		Collections.addAll(this.subsystems, subsystems);
	}
	
    public void outputToSmartDashboard() {
    	subsystems.forEach((s) -> s.outputToSmartDashboard());
    }

    public void stop() {
    	subsystems.forEach((s) -> s.stop());
    }

    public void zeroSensors() {
    	subsystems.forEach((s) -> s.zeroSensors());
    }

    public void registerEnabledLoops(Looper enabledLooper) {
    	subsystems.forEach((s) -> s.registerEnabledLoops(enabledLooper));
    }

    public void test(){
		for(Subsystem subsystem : subsystems){
			subsystem.test();
		}
	}
	
}
