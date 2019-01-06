package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map.Entry;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.wrappers.CANTalonFactory;

public class Arm extends Subsystem {
    
    private static Arm INSTANCE = null;
	
	public static Arm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }
	

	public enum WantedState {
        OFF, OPEN_LOOP
    }
    
    public enum SystemState {
        OFF, OPEN_LOOP
    }
	
	private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;
	
	
	private final TalonSRX rotationMotor;
	private final TalonSRX extensionMotor;
	
	private final AnalogPotentiometer rotationPot;
	private final AnalogPotentiometer extensionPot;
	
	private boolean stateChanged = false;
	private double currentStateStartTime;
	
	private ArmInput openLoopInput = new ArmInput(0d,0d);
	
	
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
                case OFF:
                default:
                    newState = handleOff(timestamp);
                    break;
            }
            if (newState != systemState) {
                System.out.println("Arm state " + systemState + " to " + newState);
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
	
	public Arm() {
		rotationMotor = CANTalonFactory.createDefaultTalon(RobotMap.ARM_ROTATION_CAN_ID);
		rotationMotor.setNeutralMode(NeutralMode.Brake);
	
		extensionMotor = CANTalonFactory.createDefaultTalon(RobotMap.ARM_EXTENSION_CAN_ID);
		extensionMotor.setNeutralMode(NeutralMode.Brake);
		
		rotationPot = new AnalogPotentiometer(RobotMap.ARM_ROTATION_POT_CHANNEL);
		extensionPot = new AnalogPotentiometer(RobotMap.ARM_EXTENSION_POT_CHANNEL);
    }
    
    @Override
    public void outputToSmartDashboard() {
		SmartDashboard.putNumber("armPot",rotationPot.get());
		SmartDashboard.putNumber("extPot",extensionPot.get());
    }
	
	private SystemState handleOpenLoop(double timestamp) {
		extensionMotor.set(ControlMode.PercentOutput,openLoopInput.getExtensionInput());
		rotationMotor.set(ControlMode.PercentOutput,openLoopInput.getRotationInput());
		
		return defaultStateTransfer(timestamp);
	}
	
	private SystemState handleOff(double timestamp) {
		extensionMotor.set(ControlMode.PercentOutput,0);
		rotationMotor.set(ControlMode.PercentOutput,0);
		
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
    public void stop() {
        wantedState = WantedState.OFF;
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
	
	public void reset() {
		systemState = SystemState.OFF;
	}

	@Override
	public void test() {
	}
	
	public void setOpenLoopInput(ArmInput armInput){
		synchronized (openLoopInput){
			openLoopInput = armInput;
		}
	}
	
	public SystemState getSystemState() {
		return systemState;
	}
	
}