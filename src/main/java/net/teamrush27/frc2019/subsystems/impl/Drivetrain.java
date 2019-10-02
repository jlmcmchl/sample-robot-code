package net.teamrush27.frc2019.subsystems.impl;

import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Drivetrain extends Subsystem {

  private static final Logger LOG = LogManager.getLogger(Drivetrain.class);
  private static final String TAG = "DRIVETRAIN";

  private static Drivetrain INSTANCE = null;

  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  private DriveMode driveMode;
  
  private PeriodicIO periodicIO;

  private double timeSinceModeSwitch;
  private boolean modeChanged;
  
  private final Loop loop = new Loop() {
    private DriveMode lastMode;

    @Override
    public void onStart(double timestamp) {
      synchronized (Drivetrain.this) {

      }
    }

    @Override
    public void onLoop(double timestamp) {
      synchronized (Drivetrain.this) {
        if (driveMode != lastMode) {
          LOG.info("DriveMode changed from {} to {}", lastMode, driveMode);
          timeSinceModeSwitch = timestamp;
          lastMode = driveMode;
          modeChanged = true;
        } else {
          modeChanged = false;
        }

        switch (driveMode) {
          default:
            LOG.warn("Unexpected drive mode: " + driveMode);
            break;
        }
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

  public Drivetrain() {
    periodicIO = new PeriodicIO();
  }

  @Override
  public void outputToSmartDashboard() {

  }

  @Override
  public void stop() {

  }

  @Override
  public void zeroSensors() {

  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void test() {
  }

  @Override
  public synchronized void readPeriodicInputs() {
    periodicIO.timestamp = Timer.getFPGATimestamp();
  }

  @Override
  public synchronized void writePeriodicOutputs() {

  }

  @Override
  public String id() {
    return TAG;
  }

  public static class PeriodicIO {
    public double timestamp;
  }
}
