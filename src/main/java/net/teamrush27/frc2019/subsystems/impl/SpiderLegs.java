package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.util.TelemetryUtil;
import net.teamrush27.frc2019.wrappers.InvertableDigitalInput;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class SpiderLegs extends Subsystem {
	
	private static Logger LOG = LogManager.getLogger(SpiderLegs.class);
	private static String TAG = "SPIDERLEGS";
	private static SpiderLegs INSTANCE = null;
	
	public static SpiderLegs getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new SpiderLegs();
		}
		return INSTANCE;
	}
	
	public enum WantedState {
		OFF, PENDING_CLIMB, CLIMB
	}
	
	private enum SystemState {
		OFF, PENDING_CLIMB, CLIMBING, CLIMBING_HOLD
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
				case PENDING_CLIMB:
					newState = handlePendingClimb(timestamp);
					break;
				case CLIMBING:
					newState = handleClimb(timestamp);
					break;
				case CLIMBING_HOLD:
					newState = handleClimbHold(timestamp);
					break;
				case OFF:
				default:
					newState = handleOff(timestamp);
					break;
			}
			if (newState != systemState) {
				LOG.info("SpiderLegs state {} to {}", systemState, newState);
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
	
	private final TalonSRX frontLegMotor;
	
	private final TalonSRX rearLegMotorMaster;
	private final TalonSRX rearLegMotorSlave;
	
	private final DigitalInput frontLegHome;
	private final DigitalInput rearLegHome;
	
	private Boolean frontHomed = false;
	private Boolean rearHomed = false;
	
	public SpiderLegs() {
		frontLegMotor = new TalonSRX(RobotMap.FRONT_SPIDER_LEG_MOTOR_CAN_ID);
		frontLegMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.setInverted(false);
		frontLegMotor.setNeutralMode(NeutralMode.Coast);
		
		rearLegMotorMaster = new TalonSRX(RobotMap.REAR_SPIDER_LEG_MOTOR_MASTER_CAN_ID);
		rearLegMotorMaster.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.setInverted(true);
		rearLegMotorMaster.setNeutralMode(NeutralMode.Coast);
		
		
		rearLegMotorSlave = new TalonSRX(RobotMap.REAR_SPIDER_LEG_MOTOR_SLAVE_CAN_ID);
		rearLegMotorSlave.configFactoryDefault();
		rearLegMotorSlave.setInverted(true);
		rearLegMotorSlave.follow(rearLegMotorMaster);
		rearLegMotorSlave.setNeutralMode(NeutralMode.Coast);
		
		
		frontLegHome = new InvertableDigitalInput(RobotMap.FRONT_SPIDER_LEG_HOME_SENSOR_ID, true);
		rearLegHome = new InvertableDigitalInput(RobotMap.REAR_SPIDER_LEG_HOME_SENSOR_ID, true);
		
		reloadGains();
	}
	
	private void reloadGains() {
		frontLegMotor.config_kP(0, 1, RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.config_kD(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.configMotionCruiseVelocity(400, RobotConstants.TALON_CONFIG_TIMEOUT);
		frontLegMotor.configMotionAcceleration(4000,RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rearLegMotorMaster.config_kP(0, 1, RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.config_kD(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.configMotionCruiseVelocity(400, RobotConstants.TALON_CONFIG_TIMEOUT);
		rearLegMotorMaster.configMotionAcceleration(4000,RobotConstants.TALON_CONFIG_TIMEOUT);
		
	}
	
	private SystemState handleOff(double timestamp) {
		rearLegMotorMaster.set(ControlMode.Disabled, 0);
		frontLegMotor.set(ControlMode.Disabled, 0);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handlePendingClimb(double timestamp) {
		frontLegMotor.set(ControlMode.MotionMagic, 4000);
		rearLegMotorMaster.set(ControlMode.MotionMagic, 7500);
		TelemetryUtil.getInstance().addEntry(Timer.getFPGATimestamp(), "spiderlegs.rear.position", String.valueOf(rearLegMotorMaster.getSelectedSensorPosition()));
		TelemetryUtil.getInstance().addEntry(Timer.getFPGATimestamp(), "spiderlegs.rear.velocity", String.valueOf(rearLegMotorMaster.getSelectedSensorVelocity()));
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleClimb(double timestamp) {
		frontLegMotor.set(ControlMode.MotionMagic, 10000);
		rearLegMotorMaster.set(ControlMode.MotionMagic, 12750);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleClimbHold(double timestamp) {
//		frontLegMotor.set(ControlMode.MotionMagic, 10000);
//		rearLegMotorMaster.set(ControlMode.MotionMagic, 12350);
		if(stateChanged) {
			setVoltageCompensation(true);
		}
		
		return defaultStateTransfer(timestamp);
	}
	
	private void setVoltageCompensation(boolean enabled) {
		if(enabled){
			frontLegMotor.configVoltageCompSaturation(3);
			frontLegMotor.enableVoltageCompensation(true);
		} else {
			frontLegMotor.enableVoltageCompensation(false);
		}
	}
	
	
	private SystemState defaultStateTransfer(double timestamp) {
		switch (wantedState) {
			case PENDING_CLIMB:
				return SystemState.PENDING_CLIMB;
			case CLIMB:
				return SystemState.CLIMBING;
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
	public void readPeriodicInputs(){
	}
	
	
	@Override
	public void outputToSmartDashboard() {
		LOG.trace("front {} {} - rear {} {}", frontLegHome.get(), frontLegMotor.getSelectedSensorPosition(),rearLegHome.get(), rearLegMotorMaster.getSelectedSensorPosition());
		
		SmartDashboard
			.putNumber("spiderlegs.front.position", frontLegMotor.getSelectedSensorPosition());
		SmartDashboard
			.putNumber("spiderlegs.rear.position", rearLegMotorMaster.getSelectedSensorPosition());
		SmartDashboard.putBoolean("spiderlegs.front.home", frontLegHome.get());
		SmartDashboard.putBoolean("spiderlegs.rear.home", rearLegHome.get());
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.OFF;
	}
	
	@Override
	public void zeroSensors() {
		if(!frontHomed){
			if(frontLegHome.get()){
				frontLegMotor.setSelectedSensorPosition(0);
				frontHomed = true;
				LOG.info("front spiderlegs homed");
			}
		}
		
		if(!rearHomed){
			if(rearLegHome.get()){
				rearLegMotorMaster.setSelectedSensorPosition(0);
				rearHomed = true;
				LOG.info("rear spiderlegs homed");
			}
		}
	}
	
	@Override
	public void test() {
	
	}
	
	@Override
	public String id(){
		return TAG;
	}
	
	public void setWantedState(WantedState wantedState) {
		synchronized (wantedState) {
			this.wantedState = wantedState;
		}
	}
}
