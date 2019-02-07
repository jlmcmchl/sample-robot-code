package net.teamrush27.frc2019.subsystems.impl;

import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Arm extends Subsystem {
	
	private static final String TAG = "ARM";
	private static final Logger LOG = LogManager.getLogger(Arm.class);
	private static Arm INSTANCE = null;
	
	public static Arm getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Arm();
		}
		return INSTANCE;
	}
	
	public enum WantedState {
		OFF, OPEN_LOOP, CLOSED_LOOP
	}
	
	public enum SystemState {
		OFF, OPEN_LOOP, CLOSED_LOOP
	}
	
	private WantedState wantedState = WantedState.OFF;
	private SystemState systemState = SystemState.OFF;
	
	private final CANSparkMax rotationMotorMaster;
	private final CANSparkMax rotationMotorSlave;
	private final CANSparkMax extensionMotor;
	
	private final DigitalInput rotationHomeSensor;
	private final DigitalInput extensionHomeSensor;
	
	private boolean stateChanged = false;
	private double currentStateStartTime;
	
	private ArmInput openLoopInput = new ArmInput(0d, 0d);
	private ArmInput closedLoopInput = new ArmInput(0d, 0d);
	
	private ArmState armState = new ArmState(0, 0, false, false);
	
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
				LOG.info("Arm state {} to {}",systemState,newState);
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
	
	public Arm() {
		rotationMotorMaster = new CANSparkMax(RobotMap.ARM_ROTATION_MASTER_CAN_ID,
			MotorType.kBrushless);
		rotationMotorMaster.restoreFactoryDefaults();
		rotationMotorMaster.setIdleMode(IdleMode.kBrake);
		rotationMotorMaster.setSmartCurrentLimit(50);
		rotationMotorMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
		rotationMotorMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
		
		rotationMotorSlave = new CANSparkMax(RobotMap.ARM_ROTATION_SLAVE_CAN_ID,
			MotorType.kBrushless);
		rotationMotorSlave.restoreFactoryDefaults();
		rotationMotorSlave.setSmartCurrentLimit(50);
		rotationMotorSlave.follow(rotationMotorMaster);
		
		extensionMotor = new CANSparkMax(RobotMap.ARM_EXTENSION_CAN_ID, MotorType.kBrushless);
		extensionMotor.setIdleMode(IdleMode.kBrake);
		extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
		extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
		extensionMotor.setOpenLoopRampRate(.1);
		extensionMotor.setClosedLoopRampRate(.1);
		extensionMotor.setSmartCurrentLimit(50);
		
		rotationHomeSensor = new DigitalInput(RobotMap.ARM_ROTATION_HOME_SENSOR_ID);
		extensionHomeSensor = new DigitalInput(RobotMap.ARM_EXTENSION_HOME_SENSOR_ID);
		
		reloadGains();
	}
	
	private void reloadGains() {
		rotationMotorMaster.getPIDController().setP(1 / 2500d, 0);
		rotationMotorMaster.getPIDController().setI(0, 0);
		rotationMotorMaster.getPIDController().setD(.01, 0);
		rotationMotorMaster.getPIDController().setFF(1d / 6000d, 0);
		rotationMotorMaster.getPIDController().setOutputRange(-.5, .5, 0);
		rotationMotorMaster.getPIDController()
			.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		rotationMotorMaster.getPIDController().setSmartMotionAllowedClosedLoopError(1, 0);
		rotationMotorMaster.getPIDController().setSmartMotionMaxAccel(360, 0);
		rotationMotorMaster.getPIDController().setSmartMotionMaxVelocity(180, 0);
		rotationMotorMaster.getPIDController().setSmartMotionMinOutputVelocity(1, 0);
		
		extensionMotor.getPIDController().setP(.1, 0);
		extensionMotor.getPIDController().setI(0, 0);
		extensionMotor.getPIDController().setD(0, 0);
		extensionMotor.getPIDController().setOutputRange(-.75, .75, 0);
		extensionMotor.getPIDController()
			.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		extensionMotor.getPIDController().setSmartMotionAllowedClosedLoopError(.1, 0);
		extensionMotor.getPIDController().setSmartMotionMaxAccel(93, 0);
		extensionMotor.getPIDController().setSmartMotionMaxVelocity(46, 0);
		extensionMotor.getPIDController().setSmartMotionMinOutputVelocity(1, 0);
	}
	
	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("arm.rotation", armState.getRotation());
		SmartDashboard.putBoolean("arm.rotation.reset", armState.isRotationAtHome());
		SmartDashboard.putNumber("arm.extension", armState.getExtension());
		SmartDashboard.putBoolean("arm.extension.reset", armState.isExtensionAtHome());
	}
	
	private SystemState handleClosedLoop(double timestamp) {
		extensionMotor.getPIDController()
			.setReference(closedLoopInput.getExtensionInput(), ControlType.kSmartMotion);
		rotationMotorMaster.getPIDController()
			.setReference(closedLoopInput.getRotationInput(), ControlType.kSmartMotion);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleOpenLoop(double timestamp) {
		Double rotationInput = openLoopInput.getRotationInput();
		Double extensionInput = openLoopInput.getExtensionInput();
		
		extensionMotor.set(extensionInput);
		rotationMotorMaster.set(rotationInput);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleOff(double timestamp) {
		extensionMotor.set(0);
		rotationMotorMaster.set(0);
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState defaultStateTransfer(double timestamp) {
		switch (wantedState) {
			case OPEN_LOOP:
				return SystemState.OPEN_LOOP;
			case OFF:
			default:
				return SystemState.OFF;
		}
	}
	
	@Override
	public void readPeriodicInputs() {
		armState = new ArmState( //
			rotationMotorMaster.getEncoder().getPosition(), //
			extensionMotor.getEncoder().getPosition(), //
			rotationHomeSensor.get(), //
			extensionHomeSensor.get());
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.OFF;
	}
	
	@Override
	public void zeroSensors() {
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}
	
	@Override
	public void test() {
	}
	
	public void setOpenLoopInput(ArmInput armInput) {
		synchronized (openLoopInput) {
			openLoopInput = armInput;
		}
	}
	
	public void setClosedLoopInput(ArmInput armInput) {
		synchronized (closedLoopInput) {
			closedLoopInput = armInput;
		}
	}
	
	
	private class ArmState {
		
		// in degrees from vertical
		private final double rotation;
		// in inches
		private final double extension;
		
		private final boolean rotationAtHome;
		private final boolean extensionAtHome;
		
		public ArmState(double rotation, double extension, boolean rotationAtHome,
			boolean extensionAtHome) {
			this.rotation = rotation;
			this.extension = extension;
			this.rotationAtHome = rotationAtHome;
			this.extensionAtHome = extensionAtHome;
		}
		
		public double getRotation() {
			return rotation;
		}
		
		public double getExtension() {
			return extension;
		}
		
		public boolean isRotationAtHome() {
			return rotationAtHome;
		}
		
		public boolean isExtensionAtHome() {
			return extensionAtHome;
		}
	}
}