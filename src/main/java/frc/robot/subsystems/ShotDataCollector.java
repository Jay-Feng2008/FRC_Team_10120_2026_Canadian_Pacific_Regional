package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

public final class ShotDataCollector {
  private static final List<ShotSample> samples = new ArrayList<>();

  private ShotDataCollector() {}

  public static void record(
      double distanceMeters, double flywheelRps, double wheelbackRps, double TOF) {
    samples.add(new ShotSample(distanceMeters, flywheelRps, wheelbackRps, TOF));
    System.out.printf(
        "[ShotSample] expected distance=%.3f m, flywheel=%.2f rps, wheelback=%.2f rps, TOF=%.2f%n",
        distanceMeters, flywheelRps, wheelbackRps, TOF);
  }

  public static void dumpToConsole() {
    System.out.println("==== Shot Calibration Data (Expected) ====");
    for (var s : samples) {
      System.out.printf(
          "distance=%.3f m, flywheel=%.2f rps, wheelback=%.2f rps, TOF=%.2f%n",
          s.distanceMeters, s.flywheelRps, s.wheelbackRps, s.TOF);
    }
    System.out.println("==== END ====");
  }

  public record ShotSample(
      double distanceMeters, double flywheelRps, double wheelbackRps, double TOF) {}
}
