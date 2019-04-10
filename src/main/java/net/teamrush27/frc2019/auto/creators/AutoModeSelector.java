package net.teamrush27.frc2019.auto.creators;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.modes.CharacterizeDrivetrain;
import net.teamrush27.frc2019.auto.modes.DoNothing;
import net.teamrush27.frc2019.auto.modes.RightCargo;
import net.teamrush27.frc2019.auto.modes.RightRocket;
import net.teamrush27.frc2019.auto.modes.singlepath.AltRocketReturnPathRight;
import net.teamrush27.frc2019.auto.modes.singlepath.CargoFrontToHPRight;
import net.teamrush27.frc2019.auto.modes.singlepath.CargoSideCloseToHPRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HPToCargoSideMidRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HPToRocketCloseRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HabToCargoFrontRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HabToCargoSideCloseRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HabToRocketRearRight;
import net.teamrush27.frc2019.auto.modes.singlepath.MidlineToHPRight;
import net.teamrush27.frc2019.auto.modes.singlepath.RocketRearToMidlineRight;

public class AutoModeSelector {

  public static final String AUTO_OPTIONS_DASHBOARD_KEY = "auto_modes";
  public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "selected_auto";
  public static final String ROBOT_SELECTED_AUTO_MODE_DASHBOARD_KEY = "robot_selected_auto";

  public static void update() {
    String selectedModeName = SmartDashboard
        .getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "NO SELECTED MODE!!!!");
    AutoModeCreator autoModeCreator = null;

    for (AutoModeCreator mode : ALL_MODES) {
      if (mode.getDashboardName().equals(selectedModeName)) {
        autoModeCreator = mode;
      }
    }

    if (autoModeCreator == null) {
      autoModeCreator = DEFAULT_MODE;
    }

    SmartDashboard.putString(ROBOT_SELECTED_AUTO_MODE_DASHBOARD_KEY, autoModeCreator.dashboardName);
  }


  private static class AutoModeCreator {

    private final String dashboardName;
    private final Supplier<AutoModeBase> creator;

    private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator) {
      this.dashboardName = dashboardName;
      this.creator = creator;
    }

    public Supplier<AutoModeBase> getCreator() {
      return creator;
    }

    public String getDashboardName() {
      return dashboardName;
    }
  }

  private static final AutoModeCreator DEFAULT_MODE = new AutoModeCreator("Rocket",
      () -> new RightRocket());

  private static final AutoModeCreator[] ALL_MODES = {
      new AutoModeCreator("Cargo Ship", () -> new RightCargo()),
      new AutoModeCreator("Rocket", () -> new RightRocket()),
      new AutoModeCreator("Nothing", () -> new DoNothing()),
      new AutoModeCreator("Characterize", () -> new CharacterizeDrivetrain()),
      new AutoModeCreator("HabToCargoSideCloseRight", () -> new HabToCargoSideCloseRight()),
      new AutoModeCreator("CargoSideCloseToHPRight", () -> new CargoSideCloseToHPRight()),
      new AutoModeCreator("HPToCargoSideMidRight", () -> new HPToCargoSideMidRight()),
      new AutoModeCreator("HabToRocketRearRight", () -> new HabToRocketRearRight()),
      new AutoModeCreator("RocketRearToMidlineRight", () -> new RocketRearToMidlineRight()),
      new AutoModeCreator("MidlineToHPRight", () -> new MidlineToHPRight()),
      new AutoModeCreator("HPToRocketSideCloseRight", () -> new HPToRocketCloseRight()),
      new AutoModeCreator("AltRocketReturnPathRight", () -> new AltRocketReturnPathRight()),
      new AutoModeCreator("HabToCargoFrontRight", () -> new HabToCargoFrontRight()),
      new AutoModeCreator("CargoFrontToHPRight", () -> new CargoFrontToHPRight()),
  };


  public static void initAutoModeSelector() {

    List<String> modeNames = new ArrayList<String>();

    for (AutoModeCreator mode : ALL_MODES) {
      modeNames.add(mode.getDashboardName());
    }

    final Gson gson = new Gson();

    String modesJSON = gson.toJson(modeNames);

    SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, modesJSON);
    SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, DEFAULT_MODE.getDashboardName());
  }

  public static AutoModeBase getSelectedAutoMode() {
    String selectedModeName = SmartDashboard
        .getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "NO SELECTED MODE!!!!");
    for (AutoModeCreator mode : ALL_MODES) {
      if (mode.getDashboardName().equals(selectedModeName)) {
        return mode.getCreator().get();
      }
    }
    DriverStation.reportError("Failed to select auto mode: " + selectedModeName, false);
    return DEFAULT_MODE.getCreator().get();
  }
}