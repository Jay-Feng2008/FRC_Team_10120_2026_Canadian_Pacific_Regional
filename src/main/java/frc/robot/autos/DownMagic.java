package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SmashBumpCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.util.AutoUtil;
import frc.robot.util.geometry.AllianceFlipUtil;

public class DownMagic extends SequentialCommandGroup {
  public DownMagic(RobotContainer robotContainer) {
    PathPlannerPath down1, down2, station_intake, center_climb;
    try {
      down1 = PathPlannerPath.fromPathFile("Down1");
      station_intake = PathPlannerPath.fromPathFile("StationIntake");
      down2 = PathPlannerPath.fromPathFile("down2");
      center_climb = PathPlannerPath.fromPathFile("CenterClimb");
      // addCommands(
      //     new InstantCommand(
      //         () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));

      addCommands(
          Commands.parallel(
              AutoUtil.climbReleaseCommand(robotContainer.climb, robotContainer.flywheel),
              AutoBuilder.followPath(down1)));

      // Commands.waitSeconds(4);

      // addCommands(AutoBuilder.followPath(down2));
      addCommands(new SmashBumpCommand(robotContainer.drive));
      addCommands(
          Commands.parallel(
                  robotContainer.flywheel.runTrackTargetCommand(),
                  robotContainer.flywheel_back.runTrackTargetCommand(),
                  Commands.sequence(
                      Commands.waitSeconds(0.5),
                      Commands.parallel(
                          Commands.startEnd(
                              () -> robotContainer.feeder.setGoal(Feeder.Goal.INTAKE),
                              () -> robotContainer.feeder.setGoal(Feeder.Goal.STOP),
                              robotContainer.feeder),
                          Commands.startEnd(
                              () -> robotContainer.indexer.setGoal(Indexer.Goal.INTAKE),
                              () -> robotContainer.indexer.setGoal(Indexer.Goal.STOP),
                              robotContainer.indexer))))
              .withTimeout(3));
      addCommands(AutoBuilder.followPath(station_intake));
      // addCommands(
      //     Commands.parallel(
      //             robotContainer.flywheel.runTrackTargetCommand(),
      //             robotContainer.flywheel_back.runTrackTargetCommand(),
      //             Commands.sequence(
      //                 Commands.waitSeconds(0.5),
      //                 Commands.parallel(
      //                     Commands.startEnd(
      //                         () -> robotContainer.feeder.setGoal(Feeder.Goal.INTAKE),
      //                         () -> robotContainer.feeder.setGoal(Feeder.Goal.STOP),
      //                         robotContainer.feeder),
      //                     Commands.startEnd(
      //                         () -> robotContainer.indexer.setGoal(Indexer.Goal.INTAKE),
      //                         () -> robotContainer.indexer.setGoal(Indexer.Goal.STOP),
      //                         robotContainer.indexer))))
      //         .withTimeout(5));
      // addCommands(
      //     new AutoAlignTestCommand(
      //         robotContainer.drive, new Pose2d(15.7, 5.1, Rotation2d.fromDegrees(180)), false));
      addCommands(AutoBuilder.followPath(center_climb));
      addCommands(
          new InstantCommand(
                  () -> {
                    robotContainer.climb.runVolts(4);
                  })
              .withTimeout(2));
      addCommands(
          new InstantCommand(
              () -> {
                double ang = robotContainer.climb.getAngleRads();
                robotContainer.climb.runGoal(ang, 0.0, 0.0, 0.0);
              }));

      // addCommands(new MegaTrackIterativeCommand(robotContainer, false).withTimeout(4));
      // addCommands(
      //     new InstantCommand(
      //         () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));

      // addCommands(new MegaTrackIterativeCommand(robotContainer, false));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  public static Pose2d getStartPose(Alliance alliance) {
    Pose2d bluePose2d = new Pose2d(4.59, 0.51, new Rotation2d(Degrees.of(90)));
    if (alliance == Alliance.Blue) return bluePose2d;
    else return AllianceFlipUtil.apply(bluePose2d);
  }
}
