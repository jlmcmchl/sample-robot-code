package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Gripper extends Subsystem {
	
	private static final String TAG = "GRIPPER";
	private static final Logger LOG = LogManager.getLogger(Gripper.class);
	private static Gripper INSTANCE = null;
	
	public static Gripper getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Gripper();
		}
		return INSTANCE;
	}
	
	public enum WantedState {
		OFF, INTAKE_CARGO, EXHAUST_CARGO, INTAKE_HATCH, EXHAUST_HATCH
	}
	
	private enum SystemState {
		OFF, INTAKE_CARGO, HOLD_CARGO, EXHAUST_CARGO, INTAKE_HATCH, HOLD_HATCH
	}
	
	private WantedState wantedState = WantedState.OFF;
	private SystemState systemState = SystemState.OFF;
	
	private boolean hasGamepiece = false;
	private boolean stateChanged = false;
	private double currentStateStartTime;
	
	private Loop loop = new Loop() {
		
		@Override
		public void onStart(double timestamp) {
			currentStateStartTime = timestamp;
		}
		
		@Override
		public void onLoop(double timestamp) {
			SystemState newState;
			switch (systemState) {
				case INTAKE_CARGO:
					newState = handleIntakeCargo(timestamp);
					break;
				case HOLD_CARGO:
					newState = handleHoldCargo(timestamp);
					break;
				case EXHAUST_CARGO:
					newState = handleExhaustCargo(timestamp);
					break;
				case INTAKE_HATCH:
					newState = handleIntakeHatch(timestamp);
					break;
				case HOLD_HATCH:
					newState = handleHoldHatch(timestamp);
					break;
				case OFF:
				default:
					newState = handleOff(timestamp);
					break;
			}
			if (newState != systemState) {
				LOG.info("Gripper state {} to {}", systemState, newState);
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
		
		@Override
		public String id() {
			return TAG;
		}
	};
	
	
	private final TalonSRX gripperMotorTop;
	private final TalonSRX gripperMotorBottom;
	
	private final Servo topJawServo;
	private final Servo bottomJawServo;
	
	private final AnalogInput detective;
	private final DigitalInput whatchman;
	
	public Gripper() {
		gripperMotorTop = new TalonSRX(RobotMap.GRIPPER_MOTOR_MASTER_CAN_ID);
		gripperMotorTop.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperMotorTop.configOpenloopRamp(.1, RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperMotorTop.configContinuousCurrentLimit(30, RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperMotorTop.enableCurrentLimit(true);
		
		gripperMotorBottom = new TalonSRX(RobotMap.GRIPPER_MOTOR_SLAVE_CAN_ID);
		gripperMotorBottom.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperMotorBottom.configOpenloopRamp(.1, RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperMotorBottom.configContinuousCurrentLimit(30, RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperMotorBottom.enableCurrentLimit(true);
		gripperMotorBottom.follow(gripperMotorTop);
		
		
		topJawServo = new Servo(RobotMap.GRIPPER_JAWS_TOP_SERVO_ID);
		bottomJawServo = new Servo(RobotMap.GRIPPER_JAWS_BOTTOM_SERVO_ID);
		
		detective = new AnalogInput(RobotMap.GRIPPER_CARGO_ANALOG_SENSOR_ID);
		whatchman = new DigitalInput(RobotMap.GRIPPER_HATCH_DIGITAL_SENSOR_ID);
	}
	
	
	private SystemState handleOff(double timestamp) {
		gripperMotorTop.set(ControlMode.Disabled, 0);
		topJawServo.set(1);
		bottomJawServo.set(1);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleHoldHatch(double timestamp) {
		if(timestamp - currentStateStartTime > .05){
			bottomJawServo.set(.3875);
		}
		
		topJawServo.set(.3875);
		
		if(WantedState.INTAKE_HATCH.equals(wantedState)){
			return SystemState.HOLD_HATCH;
		}
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleIntakeHatch(double timestamp) {
		bottomJawServo.set(.5);
		topJawServo.set(.5);

		if (!whatchman.get()) {
			return SystemState.HOLD_HATCH;
		}
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleExhaustCargo(double timestamp) {
		firstFoundBall=0;
		gripperMotorTop.set(ControlMode.PercentOutput, -1);
		
		if(WantedState.EXHAUST_CARGO.equals(wantedState) && detective.getVoltage() < 1.5){
			wantedState = WantedState.OFF;
			return SystemState.OFF;
		}
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleHoldCargo(double timestamp) {
		firstFoundBall = 0;
		gripperMotorTop.set(ControlMode.PercentOutput, .1);
		
		if(WantedState.EXHAUST_CARGO.equals(wantedState)){
			return SystemState.EXHAUST_CARGO;
		}
		
		if(WantedState.INTAKE_CARGO.equals(wantedState) && detective.getVoltage() > 1){
			return SystemState.HOLD_CARGO;
		}
		
		return defaultStateTransfer(timestamp);
	}
	
	private double firstFoundBall = 0;
	
	private SystemState handleIntakeCargo(double timestamp) {
		gripperMotorTop.set(ControlMode.PercentOutput, 1);
		
		if(WantedState.INTAKE_CARGO.equals(wantedState) && detective.getVoltage() > 1.5){
			if(firstFoundBall == 0) {
				firstFoundBall = Timer.getFPGATimestamp();
			}
			if(Timer.getFPGATimestamp() - firstFoundBall > .1){
				return SystemState.HOLD_CARGO;
			}
		}
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState defaultStateTransfer(double timestamp) {
		switch (wantedState) {
			case INTAKE_CARGO:
				return SystemState.INTAKE_CARGO;
			case EXHAUST_CARGO:
				return SystemState.EXHAUST_CARGO;
			case INTAKE_HATCH:
				return SystemState.INTAKE_HATCH;
			default:
			case OFF:
			case EXHAUST_HATCH:
				return SystemState.OFF;
		}
		
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	int i = 0;
	
	@Override
	public void outputToSmartDashboard() {
		if(SystemState.HOLD_CARGO.equals(systemState) || SystemState.HOLD_HATCH.equals(systemState)){
			LED.getInstance().setHasGamePiece(true);
		} else {
			hasGamepiece = false;
			LED.getInstance().setHasGamePiece(false);
		}
		if(SystemState.INTAKE_HATCH.equals(systemState) || SystemState.INTAKE_CARGO.equals(systemState)){
			LED.getInstance().setIntaking(true);
		} else {
			LED.getInstance().setIntaking(false);
		}
		if(SystemState.EXHAUST_CARGO.equals(systemState)){
			LED.getInstance().setExhausting(true);
		} else {
			LED.getInstance().setExhausting(false);
		}

		SmartDashboard.putNumber("detective", detective.getVoltage());
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.OFF;
	}
	
	@Override
	public void zeroSensors() {
	
	}
	
	@Override
	public void test() {
	
	}

	@Override
	public void writePeriodicOutputs() {
		//System.out.println(detective.getVoltage());
		//System.out.println(whatchman.get());
	}
	
	public void setWantedState(WantedState wantedState) {
		synchronized (wantedState) {
			this.wantedState = wantedState;
		}
	}
	
	public void transitionCargo() {
		synchronized (wantedState) {
			switch (wantedState) {
				case INTAKE_CARGO:
					wantedState = WantedState.EXHAUST_CARGO;
					break;
				case EXHAUST_CARGO:
					wantedState = WantedState.OFF;
					break;
				default:
					wantedState = WantedState.INTAKE_CARGO;
			}
		}
	}
	
	public void transitionHatch() {
		synchronized (systemState) {
			switch (systemState) {
				default:
				case OFF:
					wantedState = WantedState.INTAKE_HATCH;
					break;
				case INTAKE_HATCH:
					wantedState = WantedState.INTAKE_HATCH;
					systemState = SystemState.HOLD_HATCH;
					currentStateStartTime = Timer.getFPGATimestamp();
					break;
				case HOLD_HATCH:
					wantedState = WantedState.OFF;
					break;
			}
		}
	}
	
	public boolean hasGamepiece() {
		return SystemState.HOLD_CARGO.equals(systemState) || SystemState.HOLD_HATCH.equals(systemState);
	}
	
	public boolean hasHatch(){
		return SystemState.HOLD_HATCH.equals(systemState);
	}
	
	@Override
	public String id(){
		return TAG;
	}
	
}
