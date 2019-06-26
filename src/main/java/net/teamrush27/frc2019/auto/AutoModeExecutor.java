package net.teamrush27.frc2019.auto;

import net.teamrush27.frc2019.util.crash.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 */
public class AutoModeExecutor {
  private AutoModeBase autoMode;
  private Thread autoThread = null;

  private boolean isActive = false;

  public void setAutoMode(AutoModeBase new_auto_mode) {
    autoMode = new_auto_mode;
    autoThread = new Thread(new CrashTrackingRunnable() {
      @Override
      public void runCrashTracked() {
        if (autoMode != null) {
          autoMode.run();
        }
      }
    });
  }

  public void start() {
    isActive = true;
    if (autoThread != null) {
      autoThread.start();
    }
  }

  public void stop() {
    isActive = false;
    if (autoMode != null) {
      autoMode.stop();
    }

    autoThread = null;
  }

  public AutoModeBase getAutoMode() {
    return autoMode;
  }

  public boolean isActive() {
    return isActive && autoMode.isActive();
  }
}
