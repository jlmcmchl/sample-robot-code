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
import net.teamrush27.frc2019.util.ReflectingCSVWriter;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

  private final Set<Subsystem> subsystems = new HashSet<Subsystem>();
  private List<Loop> loops = new ArrayList<>();

  private ReflectingCSVWriter CSVWriter;
  private boolean logging = false;

  public SubsystemManager(Subsystem... subsystems) {
    Collections.addAll(this.subsystems, subsystems);
  }

  public SubsystemManager(List subsystems) {
  }

  public void outputToSmartDashboard() {
    subsystems.forEach((s) -> s.outputToSmartDashboard());
  }

  public void stop() {
    subsystems.forEach((s) -> s.stop());
  }

  public void zeroSensors() {
    subsystems.forEach((s) -> s.zeroSensors());
  }

  public void registerEnabledLoops(Looper enabledLooper) {
    subsystems.forEach((s) -> s.registerEnabledLoops(enabledLooper));
    enabledLooper.register(new EnabledLoop());

  }

  public void registerDisabledLoops(Looper disabledLooper) {
    disabledLooper.register(new DisabledLoop());
  }

  public void test() {
    for (Subsystem subsystem : subsystems) {
      subsystem.test();
    }
  }

  @Override
  public void register(Loop loop) {
    loops.add(loop);
  }

  private class EnabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {
      for (Loop l : loops) {
        double start = Timer.getFPGATimestamp();
        l.onStart(timestamp);
        double end = Timer.getFPGATimestamp();
        CSVWriter.add(new Profile(l.id(), "START", end, end - start));
      }
    }

    @Override
    public void onLoop(double timestamp) {
      for (Subsystem s : subsystems) {
        if (logging) {
          double start = Timer.getFPGATimestamp();
          s.readPeriodicInputs();
          double end = Timer.getFPGATimestamp();
          CSVWriter.add(new Profile(s.id(), "READ_INPUT", end, end - start));
        } else {
          s.readPeriodicInputs();
        }
      }

      for (Loop l : loops) {
        if(logging) {
          double start = Timer.getFPGATimestamp();
          l.onLoop(timestamp);
          double end = Timer.getFPGATimestamp();
          CSVWriter.add(new Profile(l.id(), "LOOP", end, end - start));
        } else {
          l.onLoop(timestamp);
        }
      }

      for (Subsystem s : subsystems) {
        if (logging) {
          double start = Timer.getFPGATimestamp();
          s.writePeriodicOutputs();
          double end = Timer.getFPGATimestamp();
          CSVWriter.add(new Profile(s.id(), "WRITE_OUTPUT", end, end - start));
        } else {
          s.writePeriodicOutputs();
        }
      }
    }

    @Override
    public void onStop(double timestamp) {
      for (Loop l : loops) {
        l.onStop(timestamp);
      }
    }

    @Override
    public String id() {
      return "SYSMGR_ENABLED_LOOP";
    }
  }

  private class DisabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {
      for (Subsystem s : subsystems) {
        if (logging) {
          double start = Timer.getFPGATimestamp();
          s.readPeriodicInputs();
          double end = Timer.getFPGATimestamp();
          CSVWriter.add(new Profile(s.id(), "READ_INPUT", end, end - start));
        } else {
          s.readPeriodicInputs();
        }
      }
      for (Subsystem s : subsystems) {
        if (logging) {
          double start = Timer.getFPGATimestamp();
          s.writePeriodicOutputs();
          double end = Timer.getFPGATimestamp();
          CSVWriter.add(new Profile(s.id(), "WRITE_OUTPUT", end, end - start));
        } else {
          s.writePeriodicOutputs();
        }
      }
    }

    @Override
    public void onStop(double timestamp) {

    }

    @Override
    public String id() {
      return "SYSMGR_DISABLED_LOOP";
    }
  }
  public void startLogging() {
    if (CSVWriter == null) {
      CSVWriter = new ReflectingCSVWriter<>("/home/lvuser/PROFILE-LOGS.csv", Profile.class);
      logging = true;
    }
  }

  public void stopLogging() {
    if (CSVWriter != null) {
      CSVWriter.flush();
      CSVWriter = null;
      logging = false;
    }
  }

  public static class Profile {
    String id;
    String action;
    double timestamp;
    double duration;

    public Profile(String id, String action, double timestamp, double duration) {
      this.id = id;
      this.action = action;
      this.timestamp = timestamp;
      this.duration = duration;
    }
  }
}
