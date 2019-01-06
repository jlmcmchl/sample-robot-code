package net.teamrush27.frc2019.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.util.crash.CrashTrackingRunnable;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List
 * object. They are started when the robot powers up and stopped after the
 * match.
 */
public class Looper {
	public final double kPeriod = RobotConstants.LOOPER_DELTA_TIME;

	private boolean running;

	private final Notifier notifier;
	private final List<Loop> loops;
	private final Object taskRunningLock = new Object();
	private double timestamp = 0;
	private double deltaTime = 0;

	private final CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			synchronized (taskRunningLock) {
				if (running) {
					double now = Timer.getFPGATimestamp();

					for (Loop loop : loops) {
						loop.onLoop(now);
					}

					deltaTime = now - timestamp;
					timestamp = now;
				}
			}
		}
	};

	public Looper() {
		notifier = new Notifier(runnable);
		running = false;
		loops = new ArrayList<>();
	}

	public synchronized void register(Loop loop) {
		synchronized (taskRunningLock) {
			loops.add(loop);
		}
	}

	public synchronized void start() {
		if (!running) {
			System.out.println("Starting loops");
			synchronized (taskRunningLock) {
				timestamp = Timer.getFPGATimestamp();
				for (Loop loop : loops) {
					loop.onStart(timestamp);
				}
				running = true;
			}
			notifier.startPeriodic(kPeriod);
		}
	}

	public synchronized void stop() {
		if (running) {
			System.out.println("Stopping loops");
			notifier.stop();
			synchronized (taskRunningLock) {
				running = false;
				timestamp = Timer.getFPGATimestamp();
				for (Loop loop : loops) {
					System.out.println("Stopping " + loop);
					loop.onStop(timestamp);
				}
			}
		}
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("looper_dt", deltaTime);
	}
}
