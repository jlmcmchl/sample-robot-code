package net.teamrush27.frc2019.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * @author team254
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */

public class LazyCANTalon extends TalonSRX {
    protected double lastValue = Double.NaN;
    protected ControlMode lastControlMode = null;
    protected int controlPeriodMs = 5;
    private int count = 0;

    public LazyCANTalon(int deviceNumber, int controlPeriodMs){
    	super(deviceNumber);
    	this.controlPeriodMs = controlPeriodMs;
	}
    
    public LazyCANTalon(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void set(ControlMode controlMode, double value) {
        if (value != lastValue || controlMode != lastControlMode) {
            lastValue = value;
            lastControlMode = controlMode;
            if(count++ % 100 == 0 || !controlMode.equals(ControlMode.Velocity)){
//				System.out.println(String.format("changed talon %s to %s - %s",getDeviceID(), controlMode.name(), value));
			}
//            setControlFramePeriod(ControlFrame.Control_3_General,controlPeriodMs);
//            setControlFramePeriod(ControlFrame.Control_4_Advanced,controlPeriodMs);
            super.set(controlMode,value);
        }
    }
}
