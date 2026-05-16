package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class AutoUtil {
  /**
   * Releases the container spring by moving the climb down for 0.3s then holding. Run in parallel
   * at auto start to expand container volume without affecting timing.
   */
  public static Command climbReleaseCommand(Climb climb, Flywheel flywheel) {
    return Commands.sequence(
        Commands.parallel(
            new RepeatCommand(new InstantCommand(() -> climb.runVolts(4))).withTimeout(0.3),
            flywheel.runFixedCommand().withTimeout(1)),
        new InstantCommand(
            () -> {
              double ang = climb.getAngleRads();
              climb.runGoal(ang, 0.0, 0.0, 0.0);
            }));
  }

  public static Command followPath(String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Load path error: " + name, e.getStackTrace());
      return Commands.none();
    }
  }

  public static Pose2d GETatrtpose(String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return path.getStartingHolonomicPose().orElse(new Pose2d());
    } catch (Exception e) {
      DriverStation.reportError("Load path error: " + name, e.getStackTrace());
      return new Pose2d();
    }
  }

  public static Command followChoreoPath(String name, boolean mirror) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);

      if (mirror) {
        path = path.mirrorPath();
      }

      return AutoBuilder.followPath(path);

    } catch (Exception e) {
      DriverStation.reportError("Load path error: " + name, e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command followChoreoPath(String name) {
    return followChoreoPath(name, false);
  }
}
