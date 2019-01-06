package net.teamrush27.frc2019.subsystems.impl;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;

public class Coordinator extends Subsystem {

	private static Coordinator INSTANCE = null;
	
	public static Coordinator getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Coordinator();
		}
		return INSTANCE;
	}
	
	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final Gripper gripper = Gripper.getInstance();
	//private final Elevator elevator = Elevator.getInstance();
	private final LED leds = LED.getInstance();
	//private final Forklift forks = Forklift.getInstance();
	private boolean flashLEDs = false;
	private int ledCycle = 0;
	
	public WantedState getWantedState() {
		return wantedState;
	}
	
	public enum WantedState {
		IDLE, AIM_TO_CUBE
	}

	public enum SystemState {
		IDLE, AIMING_TOWARDS_CUBE
	}

	private WantedState wantedState = WantedState.IDLE;
	private SystemState systemState = SystemState.IDLE;
	
	
	private boolean stateChanged = false;
	private Loop loop = new Loop() {

		private double currentStateStartTime;

		@Override
		public void onStart(double timestamp) {
			currentStateStartTime = timestamp;
		}

		@Override
		public void onLoop(double timestamp) {
			SystemState newState;
			switch (systemState) {
				case AIMING_TOWARDS_CUBE:
					newState = handleAimTowardsCube();
					break;
				case IDLE:
				default:
					newState = handleOff();
					break;
			}
			if (newState != systemState) {
				System.out.println("LED state " + systemState + " to " + newState);
				systemState = newState;
				currentStateStartTime = timestamp;
				stateChanged = true;
			} else {
				stateChanged = false;
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

	};
	
	private final NetworkTable limelightTable;
	
	public Coordinator(){
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
	}
	
	private SystemState handleAimTowardsCube() {
		
		if(Gripper.SystemState.HOLD.equals(gripper.getSystemState())){
			wantedState = WantedState.IDLE;
			return defaultStateTransfer();
		}
		
		final NetworkTableEntry tx = limelightTable.getEntry("tx");
		final NetworkTableEntry ty = limelightTable.getEntry("ty");
		final NetworkTableEntry ta = limelightTable.getEntry("ta");
		final double x = tx.getDouble(0);
		final double y = ty.getDouble(0);
		final double area = ta.getDouble(0);
		
		final double turnCommand = Math.min(Math.abs(x * .02),0.3) * Math.signum(x);
		
		final double forwardCommand;
		
		if(y > 0){
			forwardCommand = Math.min(y*.3,.3);
		} else {
			forwardCommand = 0;
		}
		
		if(y < 25){
			gripper.setWantedState(Gripper.WantedState.INTAKE);
		} else {
			gripper.setWantedState(Gripper.WantedState.OFF);
		}
		
		double leftCommand = forwardCommand + turnCommand;
		double rightCommand = forwardCommand - turnCommand;
		
		System.out.println(String.format("y : %s - l : %s - r : %s",y,leftCommand,rightCommand));
		
		drivetrain.setOpenLoop(-leftCommand,-rightCommand);
		
		return defaultStateTransfer();
	}
	
	@Override
	public void outputToSmartDashboard() {
	}
	
	private SystemState handleOff() {
		limelightTable.getEntry("ledMode").setNumber(1);
		return defaultStateTransfer();
	}
	
	private SystemState defaultStateTransfer() {
		setLEDS();
		
		switch (wantedState) {
			case AIM_TO_CUBE:
				return SystemState.AIMING_TOWARDS_CUBE;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}
	
	private void setLEDS() {
		/*if(forks.getDeployed()){
			leds.setWantedState(LED.WantedState.FORKS_DEPLOYED);
			return;
		}*/

		Gripper.SystemState gripperState = gripper.getSystemState();

		if(Gripper.SystemState.EXHAUST.equals(gripperState)){
			leds.setWantedState(LED.WantedState.EXHAUSTING_CUBE);
			return;
		}

		if(Gripper.SystemState.INTAKE.equals(gripperState)){
			leds.setWantedState(LED.WantedState.HARVESTING);
			return;
		}

		if(Gripper.SystemState.HOLD.equals(gripperState)){
			leds.setWantedState(LED.WantedState.HOLDING_CUBE);
			return;
		}
		
		if(flashLEDs){
			limelightTable.getEntry("ledMode").setNumber(2);
			if(ledCycle++ < 20){
				leds.setWantedState(LED.WantedState.HARVESTING);
				return;
			}
			if(ledCycle == 40){
				ledCycle = 0;
			}
		}

		leds.setWantedState(LED.WantedState.ENABLED);
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.IDLE;
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(loop);
	}

	public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}
	
	public boolean isFlashLEDs() {
		return flashLEDs;
	}
	
	public void setFlashLEDs(boolean flashLEDs) {
		this.flashLEDs = flashLEDs;
	}
	
	@Override
	public void test() {
	
	}
}
