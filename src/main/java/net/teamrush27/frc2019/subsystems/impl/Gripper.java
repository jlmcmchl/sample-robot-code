package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.util.math.CircularBuffer;
import net.teamrush27.frc2019.wrappers.CANTalonFactory;
import net.teamrush27.frc2019.wrappers.InvertableDigitalInput;

public class Gripper extends Subsystem {
    
    private static Gripper INSTANCE = null;
//    private final Elevator elevator = Elevator.getInstance();
    private final PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	
	public static Gripper getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Gripper();
        }
        return INSTANCE;
    }
	
	public SystemState getSystemState() {
		return systemState;
	}
	
	public boolean isUp() {
		return up;
	}
	
	public void retract() {
    	wantedState = WantedState.OFF;
	}
	
	public enum WantedState {
        OFF, INTAKE, EXHAUST, RETRACT
    }
    
    public enum SystemState {
        OFF, INTAKE, EXHAUST, HOLD, RETRACT
    }
	
	private static final double SPIN_TIME = 0.075;
	
	private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;
	
	
	private final TalonSRX gripperRightMotor;
	private final TalonSRX gripperLeftMotor;
	private final DoubleSolenoid gripperSolenoid;
	private final DigitalInput leftCubeSensor;
	private final DigitalInput rightCubeSensor;
	
	private boolean stateChanged = false;
	private double cubeLostTime = 0;
	private boolean running = false;
	private boolean noRise = true;
	private double currentStateStartTime;
	private boolean up = true;
	private boolean exhaustSlow = false;
	private double exhaustSlowSpeed = .5;
	private double spinStartTime = 0;
	private boolean spin = false;
	private CircularBuffer circularBuffer = new CircularBuffer(50);
	
	boolean reverseSpin = false;
	
	
	private Loop loop = new Loop() {
        
        @Override
        public void onStart(double timestamp) {
            currentStateStartTime = timestamp;
        }
        
        @Override
        public void onLoop(double timestamp) {
            SystemState newState;
            switch (systemState) {
                case INTAKE:
                    newState = handleIntake(timestamp);
                    break;
                case EXHAUST:
                    newState = handleExhaust(timestamp);
                    break;
				case HOLD:
					newState = handleHold(timestamp);
					break;
				case RETRACT:
					newState = handleRetract(timestamp);
					break;
                case OFF:
                default:
                    newState = handleOff(timestamp);
                    break;
            }
            if (newState != systemState) {
                System.out.println("Gripper state " + systemState + " to " + newState);
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
    
    public Gripper() {
        gripperRightMotor = CANTalonFactory.createDefaultTalon(RobotMap.GRIPPER_RIGHT_CAN_ID);
		gripperRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        gripperRightMotor.setNeutralMode(NeutralMode.Coast);
        gripperRightMotor.setInverted(true);
        
        gripperLeftMotor = CANTalonFactory.createDefaultTalon(RobotMap.GRIPPER_LEFT_CAN_ID);
        gripperLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        gripperLeftMotor.setNeutralMode(NeutralMode.Coast);
        gripperLeftMotor.setInverted(false);
        
        leftCubeSensor = new InvertableDigitalInput(RobotMap.GRIPPER_CUBE_SENSOR_LEFT_DIGITAL, true);
		rightCubeSensor = new InvertableDigitalInput(RobotMap.GRIPPER_CUBE_SENSOR_RIGHT_DIGITAL,true);
	
		gripperSolenoid = new DoubleSolenoid(RobotMap.GRIPPER_DEPLOYER_MODULE,RobotMap.GRIPPER_DEPLOYER_DOWN_CHANNEL,RobotMap.GRIPPER_DEPLOYER_UP_CHANNEL);
    }
    
    @Override
    public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("cube",leftCubeSensor.get() && rightCubeSensor.get());
    }

    private void runGripperIndividual(double lSpeed, double rSpeed) {
		gripperLeftMotor.set(ControlMode.PercentOutput, lSpeed);
		gripperRightMotor.set(ControlMode.PercentOutput, rSpeed);
		running = true;
	}
    
    private void runGripper(double speed) {
        runGripperIndividual(speed, speed);
    }
    
    private SystemState handleIntake(double timestamp) {
		if(stateChanged){
			reverseSpin = false;
		}
    	if (!spin) {
			runGripperIndividual(1, .75);

			circularBuffer.addValue(pdp.getCurrent(3));
			circularBuffer.addValue(pdp.getCurrent(4));

			if(circularBuffer.getAverage() > 30 && circularBuffer.isFull()) {
					System.out.println("starting spin");
					spin = true;
					spinStartTime = Timer.getFPGATimestamp();
					circularBuffer.clear();
			}
		} else {
			int multiplier = 1;
			
//			if(reverseSpin){
//				multiplier = -1;
//			}
    		System.out.println("spinning for " + (Timer.getFPGATimestamp() - spinStartTime));
    		runGripperIndividual(0.8 * multiplier, -0.8 * multiplier);
			
			spin = Timer.getFPGATimestamp() - spinStartTime < SPIN_TIME;
			if(!spin && !reverseSpin){
				reverseSpin = true;
			} else if(!spin && reverseSpin) {
				reverseSpin = false;
			}
		}


		if(up) {
			System.out.println("putting gripper down");
			gripperSolenoid.set(Value.kForward);
			up = false;
		}
	
		return defaultStateTransfer(timestamp);
    }
	
	private SystemState handleRetract(double timestamp) {
/*		if(elevator.isAtHome() && !up){
			up = true;
			System.out.println("putting gripper up");
			gripperSolenoid.set(Value.kReverse);
		}*/
		running = false;
		runGripper(0);
		
		return defaultStateTransfer(timestamp);
	}
	
	public synchronized void deploy() {
		System.out.println("putting gripper down");
    	gripperSolenoid.set(Value.kForward);
    	up = false;
	}
	
	boolean firstRun = true;
 
	private SystemState handleHold(double timestamp) {
		if(stateChanged){
			firstRun = false;
		}
		
    	if(timestamp - currentStateStartTime > .5){
    		if(!firstRun) {
    			firstRun = true;
    			if(!noRise) {
//    				if(elevator.isAtHome()) {
//						elevator.wantHoldPosition();
//					}
				} else {
    				noRise = false;
				}
			}
			runGripper(0.1);
		} else {
    		runGripper(1);
		}
		
		if(!leftCubeSensor.get() && !rightCubeSensor.get()){
			// lost both sensors
			if(cubeLostTime == 0){
				// above sets time when lost both sensors
				cubeLostTime = timestamp;
				return SystemState.HOLD;
			}
			
			if(timestamp - cubeLostTime > .5) {
				// if we're still lost after .5 seconds from inital loss, then consider cube lost
				cubeLostTime = 0;
				wantedState = WantedState.OFF;
				return defaultStateTransfer(timestamp);
			}
		} else {
			// if we get either sensor back, reset inital lost time
			cubeLostTime = 0;
		}
		
		switch(wantedState){
			case EXHAUST:
				return SystemState.EXHAUST;
			default:
			case OFF:
				return SystemState.HOLD;
		}
	}
	
	private SystemState handleOff(double timestamp) {
        runGripper(0);
        running = false;
		
		if(up) {
			System.out.println("putting gripper down");
			gripperSolenoid.set(Value.kForward);
			up = false;
		}
		
		return defaultStateTransfer(timestamp);
    }
    
    private SystemState handleExhaust(double timestamp) {
        if(exhaustSlow){
        	runGripper(-exhaustSlowSpeed);
		} else {
			runGripper(-1);
		}
        return defaultStateTransfer(timestamp);
    }
	
	
	public synchronized void transitionSlow() {
		exhaustSlow = true;
		wantedState = WantedState.EXHAUST;
	}
	
	private void transition() {
		switch(systemState){
			case EXHAUST:
			case INTAKE:
				wantedState =  WantedState.OFF;
				return;
			case HOLD:
				wantedState = WantedState.EXHAUST;
				return;
			case OFF:
				wantedState = WantedState.INTAKE;
				return;
		}
	}
	
	private SystemState defaultStateTransfer(double timestamp) {
    	if(SystemState.INTAKE.equals(systemState)){
    		if(leftCubeSensor.get() && rightCubeSensor.get()){
    			return SystemState.HOLD;
			}
		}
		
		if(SystemState.EXHAUST.equals(systemState)){
    		if(!leftCubeSensor.get() && !rightCubeSensor.get()){
    			if(cubeLostTime == 0){
    				cubeLostTime = timestamp;
    				return SystemState.EXHAUST;
				}
				if(timestamp - cubeLostTime > .5) {
    				cubeLostTime = 0;
					wantedState = WantedState.OFF;
					return defaultStateTransfer(timestamp);
				}
			}
		}
    	
        switch (wantedState) {
            case INTAKE:
            	return SystemState.INTAKE;
            case EXHAUST:
                return SystemState.EXHAUST;
			case RETRACT:
				return SystemState.RETRACT;
            case OFF:
            default:
            	return SystemState.OFF;
        }
    }
    
    @Override
    public void stop() {
        wantedState = WantedState.OFF;
		firstRun = false;
    }
    
    
    @Override
    public void zeroSensors() {
    }
	
	public synchronized void normalTransition() {
		exhaustSlow = false;
		transition();
	}
	
	@Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(loop);
    }
    
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }
	
	public void setSlowExhaust(boolean exhaustSlow) {
		this.exhaustSlow = exhaustSlow;
	}

	public void setSlowExhaustSpeed(double percent) {
		this.exhaustSlowSpeed = percent;
	}
	
	public void reset() {
		firstRun = true;
		systemState = SystemState.HOLD;
		up = true;
	}

	public boolean eitherSensorTriggeredOrJammed() {
		return leftCubeSensor.get() || rightCubeSensor.get() || spin;
	}
	
	@Override
	public void test() {
		gripperLeftMotor.configVoltageCompSaturation(12.0, RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperLeftMotor.enableVoltageCompensation(true);
		gripperRightMotor.configVoltageCompSaturation(12.0, RobotConstants.TALON_CONFIG_TIMEOUT);
		gripperRightMotor.enableVoltageCompensation(true);
		System.out.println("*** STARTING GRIPPER TEST ***");
		System.out.println("****** TEST 1 LEFT GRIPPER MOTOR ******");
		int SECONDS_TO_TEST = 10;
		double startTime = Timer.getFPGATimestamp();
		List<Double> currentMeasurements = new ArrayList<>();
		
		while(Timer.getFPGATimestamp() - startTime < SECONDS_TO_TEST){
			gripperLeftMotor.set(ControlMode.PercentOutput, 10.0/12.0);
			currentMeasurements.add(pdp.getCurrent(3));
		}
		OptionalDouble averageCurrent = currentMeasurements.stream().mapToDouble(a->a).average();
		System.out.println("*** AVERAGE CURRENT OVER 10 SECONDS : " + averageCurrent.getAsDouble() + "***");
		startTime = Timer.getFPGATimestamp();
		currentMeasurements = new ArrayList<>();
		
		while(Timer.getFPGATimestamp() - startTime < SECONDS_TO_TEST){
			gripperRightMotor.set(ControlMode.PercentOutput, 10.0/12.0);
			currentMeasurements.add(pdp.getCurrent(3));
		}
		averageCurrent = currentMeasurements.stream().mapToDouble(a->a).average();
		System.out.println("*** AVERAGE CURRENT OVER 10 SECONDS : " + averageCurrent.getAsDouble() + "***");
	}
}