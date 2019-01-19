package net.teamrush27.frc2019.subsystems;

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

  private final Set<Subsystem> subsystems = new HashSet<Subsystem>();
  private List<Loop> loops = new ArrayList<>();

  public SubsystemManager(Subsystem... subsystems) {
    Collections.addAll(this.subsystems, subsystems);
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
        l.onStart(timestamp);
      }
    }

    @Override
    public void onLoop(double timestamp) {
      for (Subsystem s : subsystems) {
        s.readPeriodicInputs();
      }
      for (Loop l : loops) {
        l.onLoop(timestamp);
      }
      for (Subsystem s : subsystems) {
        s.writePeriodicOutputs();
      }
    }

    @Override
    public void onStop(double timestamp) {
      for (Loop l : loops) {
        l.onStop(timestamp);
      }
    }
  }

  private class DisabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {
      for (Subsystem s : subsystems) {
        s.readPeriodicInputs();
      }
      for (Subsystem s : subsystems) {
        s.writePeriodicOutputs();
      }
    }

    @Override
    public void onStop(double timestamp) {

    }
  }
}
