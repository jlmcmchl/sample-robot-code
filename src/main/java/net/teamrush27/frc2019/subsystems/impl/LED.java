package net.teamrush27.frc2019.subsystems.impl;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
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
		DISABLED, ENABLED
	}
	
	private enum SystemState {
		DISABLED, ENABLED
	}
	
	private WantedState wantedState = WantedState.DISABLED;
	private SystemState systemState = SystemState.DISABLED;
	
	
	private boolean stateChanged = false;
	private double currentStateStartTime;
	
	private final Loop loop = new Loop() {
		private SystemState lastState = null;
		
		@Override
		public void onStart(double timestamp) {
			currentStateStartTime = timestamp;
		}
		
		@Override
		public void onLoop(double timestamp) {
			if (lastState != systemState) {
				LOG.info("LED state " + lastState + " to " + systemState);
				lastState = systemState;
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
			case DISABLED:
				return SystemState.DISABLED;
		}
	}
	
	private SystemState handleState() {
		String data = systemState.name() + ":";
		switch (systemState) {
			case ENABLED:
				data += "000";
				break;
			case DISABLED:
				data += "0000";
				break;
		}
		
		boolean success = arduino
			.transaction(data.getBytes(), data.getBytes().length, new byte[0], 0);
		LOG.trace("writing {} to arduino - success? : {}", data, success);
		return defaultStateTransfer();
	}
	
	@Override
	public void writePeriodicOutputs() {
		systemState = handleState();
	}
	
	@Override
	public void stop() {
		wantedState = WantedState.DISABLED;
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

	@Override
	public boolean checkSystem() {
		// TODO Auto-generated method stub
		return true;
	}
}
