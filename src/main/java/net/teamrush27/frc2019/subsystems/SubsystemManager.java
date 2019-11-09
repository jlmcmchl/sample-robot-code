package net.teamrush27.frc2019.subsystems;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

	public static String TAG = "SUBSYSTEMMANAGER";

	private final Set<Subsystem> subsystems = new HashSet<Subsystem>();
	private List<Loop> loops = new ArrayList<>();

	public SubsystemManager(Subsystem... subsystems) {
		Collections.addAll(this.subsystems, subsystems);
	}

	public void outputToSmartDashboard() {
		subsystems.forEach(Subsystem::outputToSmartDashboard);
	}

	public void stop() {
		subsystems.forEach(Subsystem::stop);
	}

	public void zeroSensors() {
		subsystems.forEach(Subsystem::zeroSensors);
	}

	public void registerEnabledLoops(Looper enabledLooper) {
		subsystems.forEach(s -> s.registerEnabledLoops(this));
		enabledLooper.register(new EnabledLoop());
	}

	public void registerDisabledLoops(Looper disabledLooper) {
		disabledLooper.register(new DisabledLoop());
	}

	public void test() {
		subsystems.forEach(Subsystem::test);
	}

	@Override
	public void register(Loop loop) {
		loops.add(loop);
	}

	private class EnabledLoop implements Loop {

		@Override
		public void onStart(double timestamp) {
			loops.forEach(loop -> {
				double ts = Timer.getFPGATimestamp();
				loop.onStart(ts);
			});
		}

		@Override
		public void onLoop(double timestamp) {
			readSubsystemInputs(timestamp);

			loops.forEach(loop -> {
				double ts = Timer.getFPGATimestamp();
				loop.onLoop(ts);
			});

			writeSubsystemOutputs(timestamp);
		}

		@Override
		public void onStop(double timestamp) {
			loops.forEach(loop -> {
				double ts = Timer.getFPGATimestamp();
				loop.onStop(ts);
			});
		}

		@Override
		public String id() {
			return TAG;
		}
	}

	private class DisabledLoop implements Loop {

		@Override
		public void onStart(double timestamp) {
		}

		@Override
		public void onLoop(double timestamp) {
			readSubsystemInputs(timestamp);
			loops.forEach(loop -> {
				double ts = Timer.getFPGATimestamp();
				loop.onLoop(ts);
			});
		}

		@Override
		public void onStop(double timestamp) {
		}

		@Override
		public String id() {
			return TAG;
		}
	}

	public void readSubsystemInputs(double timestamp) {
		subsystems.forEach(Subsystem::readPeriodicInputs);
	}

	public void writeSubsystemOutputs(double timestamp) {
		subsystems.forEach(Subsystem::writePeriodicOutputs);
	}

	public boolean checkSubsystems() {
		boolean result = true;

		for (Subsystem subsystem : subsystems) {
			result &= subsystem.checkSystem();
		}

		return result;
	}

	public String id() {
		return TAG;
	}
}
