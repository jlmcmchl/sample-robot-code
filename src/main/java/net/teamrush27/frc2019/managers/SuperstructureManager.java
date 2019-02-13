package net.teamrush27.frc2019.managers;

import java.util.ArrayList;
import java.util.List;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Arm.ArmState;
import net.teamrush27.frc2019.subsystems.impl.Wrist;

public class SuperstructureManager extends Subsystem {
	
	private static final String TAG = "SSMAN";
	private static SuperstructureManager INSTANCE = null;
	
	public static SuperstructureManager getInstance(){
		if(INSTANCE == null){
			INSTANCE = new SuperstructureManager();
		}
		
		return INSTANCE;
	}
	
	private Arm arm = Arm.getInstance();
	private Wrist wrist = Wrist.getInstance();
	
	public enum WantedState {
		CARGO_GROUND_PICKUP(-50d, 5d),
		HATCH_HUMAN_PICKUP(-90d, 5d),
		CARGO_SHIP(-90d, 5d),
		ROCKET_LEVEL_1(-90d, 5d),
		ROCKET_LEVEL_2(-45d, 5d),
		ROCKET_LEVEL_3(-20d, 5d),
		STOW(0d, 5d),
		CLIMB(-45d,5d);
		
		private final double rotation;
		private final double extension;
		
		WantedState(double rotation, double extension){
			this.rotation = rotation;
			this.extension = extension;
		}
		
		protected double getRotation() {
			return rotation;
		}
		
		protected double getExtension() {
			return extension;
		}
	}
	
	private WantedState wantedState = WantedState.STOW;
	private boolean wantedStateChanged = false;
	private boolean invertedRotation = false;
	private List<Command> commands = new ArrayList<>();
	
	@Override
	public void outputToSmartDashboard() {
	
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.STOW;
	}
	
	@Override
	public void zeroSensors() {
	
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}
			
			@Override
			public void onLoop(double timestamp) {
				handlePosition(timestamp);
			}
			
			@Override
			public void onStop(double timestamp) {
				stop();
			}
			
			@Override
			public String id() {
				return TAG;
			}
		});
	}
	
	private void handlePosition(double timestamp) {
		if(wantedStateChanged){
			recomputeOperations();
		}
	
		//executeCurrentOperation();
	}
	
	private void recomputeOperations() {
		ArmState armState = arm.getArmState();
		
		if(getWantedRotation() - armState.getRotationInDegrees() > 0){
			// needs to travel towards rear of robot
			
			// new setpoint is -90
			// start at 0
			// look at every angle from 0 to -90
			// at the first angle that current extension is not allowed
			// add command with extension limit at some slightly earlier angle
			// add command after with
			new Command(5d, -45d, Limit.EXTENSION, 5d);
			new Command(5d, -60d, Limit.ROTATION, 60d);
			new Command(0d, -90d, Limit.NONE, 0d);
			
			
			for(int i = armState.getRotationInDegrees().intValue(); i <= getWantedRotation(); i++){
			
			}
		
		} else if (getWantedRotation() - armState.getRotationInDegrees() < 0){
			// needs to travel towards front of robot
			for(int i = armState.getRotationInDegrees().intValue(); i >= getWantedRotation(); i--){
			
			}
		}
	}
	
	private double getWantedRotation(){
		return wantedState.getRotation() * (invertedRotation ? -1 : 1);
	}
	
	private double boundExtensionForAngle(final double angle, final double extension){
		final double flooredExtension = Math.min(extension, getMaxExtensionForAngle(angle));
		final double ceiledExtension = Math.max(flooredExtension, getMinExtensionForAngle(angle));
		return ceiledExtension;
	}
	
	private double getMaxExtensionForAngle(double angle){
		return 0d;
	}
	
	private double getMinExtensionForAngle(double angle){
		return 0d;
	}
	
	@Override
	public void test() {
	
	}
	
	@Override
	public String id(){
		return TAG;
	}
	
	
	private enum Limit{
		ROTATION, EXTENSION, NONE
	}
	
	private class Command {
		private final double extensionDemand;
		private final double rotationDemand;
		private final Limit limitType;
		private final double limit;
		
		protected Command(double extensionDemand, double rotationDemand, Limit limitType, double limit){
			this.extensionDemand = extensionDemand;
			this.rotationDemand = rotationDemand;
			this.limitType = limitType;
			this.limit = limit;
		}
		
		public double getExtensionDemand() {
			return extensionDemand;
		}
		
		public double getRotationDemand() {
			return rotationDemand;
		}
		
		public Limit getLimitType() {
			return limitType;
		}
		
		public double getLimit() {
			return limit;
		}
	}
}
