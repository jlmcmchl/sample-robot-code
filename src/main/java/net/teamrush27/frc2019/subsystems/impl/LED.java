package net.teamrush27.frc2019.subsystems.impl;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Talon;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class LED extends Subsystem {
	private static final Logger LOG = LogManager.getLogger(LED.class);
	private static final String TAG = "LED";

	private static LED INSTANCE = null;
	
	public static LED getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new LED();
		}
		return INSTANCE;
	}

	public enum WantedState {
		ENABLED, HARVESTING, HOLDING_CUBE, EXHAUSTING_CUBE, FORKS_DEPLOYED
	}

	public enum SystemState {
		
		ENABLED(0.2),
		HARVESTING(0.4),
		HOLDING_CUBE(0.6),
		EXHAUSTING_CUBE(0.8),
		FORKS_DEPLOYED(1d);
		
		private double value;
		
		SystemState(double value){
			this.value = value;
		}
		public double getValue() {
			return value;
		}
		
	}

	private WantedState wantedState = WantedState.ENABLED;
	private SystemState systemState = SystemState.ENABLED;
	
	
	private boolean stateChanged = false;
	private double currentStateStartTime;
	
	
	private final Loop loop = new Loop() {

		@Override
		public void onStart(double timestamp) {
			currentStateStartTime = timestamp;
		}

		@Override
		public void onLoop(double timestamp) {
			SystemState newState;
			
			switch(systemState){
				default:
					newState = handleState(timestamp);
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

		@Override
		public String id() {
			return TAG;
		}

	};
	
	private final I2C arduino;
	
	public LED() {
		arduino = new I2C(Port.kOnboard, 4);
	}

	@Override
	public void outputToSmartDashboard() {
	}
	
	private SystemState defaultStateTransfer() {
		
		switch (wantedState) {
			default:
			case ENABLED:
				return SystemState.ENABLED;
			case HARVESTING:
				return SystemState.HARVESTING;
			case HOLDING_CUBE:
				return SystemState.HOLDING_CUBE;
			case EXHAUSTING_CUBE:
				return SystemState.EXHAUSTING_CUBE;
			case FORKS_DEPLOYED:
				return SystemState.FORKS_DEPLOYED;
		}
	}
	
	private SystemState handleState(double timestamp) {
		arduino.transaction(systemState.name().getBytes(), systemState.name().getBytes().length, null, 0);
		
		return defaultStateTransfer();
	}

	@Override
	public void stop() {
		wantedState = WantedState.ENABLED;
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
}
