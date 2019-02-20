package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Wrist extends Subsystem {
	
	private static Logger LOG = LogManager.getLogger(Wrist.class);
	private static String TAG = "WRIST";
	private static Wrist INSTANCE = null;
	
	private static final int MAX_PWM = 2188;
	private static final int MID_PWM = 1232;
	private static final int MIN_PWM = 314;
	
	public static Wrist getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Wrist();
		}
		return INSTANCE;
	}
	
	public enum WantedState {
		OFF, OPEN_LOOP, CLOSED_LOOP
	}
	
	private enum SystemState {
		OFF, OPEN_LOOP, CLOSED_LOOP, HOLD
	}
	
	private WantedState wantedState = WantedState.OFF;
	private SystemState systemState = SystemState.OFF;
	
	private boolean zeroed = false;
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
				case OPEN_LOOP:
					newState = handleOpenLoop(timestamp);
					break;
				case CLOSED_LOOP:
					newState = handleClosedLoop(timestamp);
					break;
				case HOLD:
					newState = handleHold(timestamp);
					break;
				case OFF:
				default:
					newState = handleOff(timestamp);
					break;
			}
			if (newState != systemState) {
				LOG.info("Wrist state {} to {}", systemState, newState);
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
	
	private final TalonSRX wristMotor;
	private final CANifier wristSensor;
	
	private Double openLoopInput = 0d;
	private Double closedLoopInput = 0d;
	
	public Wrist() {
		wristMotor = new TalonSRX(RobotMap.WRIST_MOTOR_CAN_ID);
		wristMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configVoltageCompSaturation(4, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.enableVoltageCompensation(false);
		
		wristMotor.configRemoteFeedbackFilter(RobotMap.WRIST_CANIFIER_CAN_ID, RemoteSensorSource.CANifier_Quadrature, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0 ,RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.setSensorPhase(true);
		
		wristSensor = new CANifier(RobotMap.WRIST_CANIFIER_CAN_ID);
		wristSensor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		
		reloadGains();
	}
	
	private void reloadGains() {
		wristMotor.config_kP(0, 5, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.config_kD(0, .1, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configMotionAcceleration(degreesToTicks(180) / 10,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configMotionCruiseVelocity(degreesToTicks(90) / 10,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		
	}
	
	private SystemState handleOff(double timestamp) {
		wristMotor.set(ControlMode.Disabled, 0);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleClosedLoop(double timestamp) {
		if(stateChanged){
			wristMotor.configVoltageCompSaturation(6);
		}
		wristMotor.set(ControlMode.MotionMagic, degreesToTicks(closedLoopInput));
		
		if(WantedState.CLOSED_LOOP.equals(wantedState) && Math.abs(getEncoderAngle() - closedLoopInput) < 3){
			return SystemState.HOLD;
		} else {
			return defaultStateTransfer(timestamp);
		}
	}
	
	private int degreesToTicks(double degrees){
		return Double.valueOf(((degrees / 90d) * 1024d)).intValue();
	}
	
	private SystemState handleHold(double timestamp) {
		if(stateChanged){
			wristMotor.configVoltageCompSaturation(4);
		}
		
		
		if(WantedState.CLOSED_LOOP.equals(wantedState) && Math.abs(getEncoderAngle() - closedLoopInput) < 3){
			return SystemState.HOLD;
		} else {
			return defaultStateTransfer(timestamp);
		}
	}
	
	private SystemState handleOpenLoop(double timestamp) {
		wristMotor.set(ControlMode.PercentOutput, openLoopInput);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState defaultStateTransfer(double timestamp) {
		switch (wantedState) {
			case OPEN_LOOP:
				return SystemState.OPEN_LOOP;
			case CLOSED_LOOP:
				return SystemState.CLOSED_LOOP;
			default:
			case OFF:
				return SystemState.OFF;
		}
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	
	@Override
	public void outputToSmartDashboard() {
//		double[] array = new double[2];
//		wristSensor.getPWMInput(PWMChannel.PWMChannel0, array);
//		LOG.info("{} {}", getEncoderAngle(), array[0]);

		double[] array = new double[2];
		wristSensor.getPWMInput(PWMChannel.PWMChannel0, array);
		double value = array[0];

		SmartDashboard.putNumber("wrist.position", getEncoderAngle());
		SmartDashboard.putNumber("wrist.pwm", array[0]);
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.OFF;
	}
	
	@Override
	public void zeroSensors() {
		if(!zeroed) {
			wristSensor
				.setQuadraturePosition(-Double.valueOf((getPWMAngle() / 90) * 1024).intValue(),
					RobotConstants.TALON_CONFIG_TIMEOUT);
			LOG.info("INITIAL POSITION {} {}", getPWMAngle(),getEncoderAngle());
			zeroed = true;
		}
	}
	
	public double getEncoderAngle(){
		return ((wristMotor.getSelectedSensorPosition()*1d) / 1024d) * 90d;
	}
	
	public double getPWMAngle(){
		double[] array = new double[2];
		wristSensor.getPWMInput(PWMChannel.PWMChannel0, array);
		double value = array[0];
		
		return (((value - MID_PWM) / (MAX_PWM - MIN_PWM))*-1)*180;
	}
	
	@Override
	public void test() {
	
	}
	
	public void setOpenLoopInput(double openLoopInput) {
		synchronized (this.openLoopInput) {
			this.openLoopInput = openLoopInput;
		}
	}
	
	public void setClosedLoopInput(double closedLoopInput) {
		synchronized (this.closedLoopInput) {
			closedLoopInput = Math.max(closedLoopInput, -90);
			closedLoopInput = Math.min(closedLoopInput, 90);
			this.closedLoopInput = closedLoopInput;
		}
	}
	
	public void setWantedState(WantedState wantedState) {
		synchronized (wantedState) {
			this.wantedState = wantedState;
		}
	}
	
	@Override
	public String id(){
		return TAG;
	}
}
