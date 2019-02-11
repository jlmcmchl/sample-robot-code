package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
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
		OFF, OPEN_LOOP, CLOSED_LOOP
	}
	
	private WantedState wantedState = WantedState.OFF;
	private SystemState systemState = SystemState.OFF;
	
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
		wristMotor.enableVoltageCompensation(true);
		
		wristMotor.configRemoteFeedbackFilter(RobotMap.WRIST_CANIFIER_CAN_ID, RemoteSensorSource.CANifier_Quadrature, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0 ,RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.setSensorPhase(true);
		
		wristSensor = new CANifier(RobotMap.WRIST_CANIFIER_CAN_ID);
		wristSensor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		
		reloadGains();
	}
	
	private void reloadGains() {
		wristMotor.config_kP(0, 10, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.config_kD(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configMotionAcceleration(((4096 / 360) * 180) / 10,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		wristMotor.configMotionCruiseVelocity(((4096 / 360) * 90) / 10,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		
	}
	
	private SystemState handleOff(double timestamp) {
		wristMotor.set(ControlMode.Disabled, 0);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleClosedLoop(double timestamp) {
		wristMotor.set(ControlMode.Position, 0);
		
		return defaultStateTransfer(timestamp);
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
		LOG.trace("wrist.position : {}",wristMotor.getSelectedSensorPosition());
		
		SmartDashboard.putNumber("wrist.position", wristMotor.getSelectedSensorPosition());
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.OFF;
	}
	
	@Override
	public void zeroSensors() {
		wristSensor.setQuadraturePosition(0, RobotConstants.TALON_CONFIG_TIMEOUT);
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
