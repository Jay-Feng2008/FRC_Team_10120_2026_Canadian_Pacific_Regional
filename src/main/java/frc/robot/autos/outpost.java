package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakearm.Intakearm;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.wheelback.Wheelback;
import frc.robot.util.AutoUtil;
import frc.robot.util.geometry.AllianceFlipUtil;

public class outpost {
  public static Command build(
      Drive drive,
      Intake intake,
      Flywheel flywheel,
      Wheelback flywheel_back,
      Indexer indexer,
      Feeder feeder,
      Climb climb,
      Intakearm intakearm) {

    return Commands.sequence(
        Commands.parallel(
            AutoUtil.climbReleaseCommand(climb, flywheel),
            Commands.sequence(
                new InstantCommand(
                    () ->
                        drive.setPose(
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                ? AutoUtil.GETatrtpose("GotoStation") // 蓝方起始位置
                                : AllianceFlipUtil.apply(AutoUtil.GETatrtpose("GotoStation")))),
                AutoUtil.followPath("GotoStation"))),
        Commands.waitSeconds(1),
        AutoUtil.followPath("AimHub"),
        Commands.parallel(
            DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0),
            flywheel.runTrackTargetCommand(),
            flywheel_back.runTrackTargetCommand(),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.parallel(
                    Commands.startEnd(
                        () -> feeder.setGoal(Feeder.Goal.INTAKE),
                        () -> feeder.setGoal(Feeder.Goal.STOP),
                        feeder),
                    Commands.startEnd(
                        () -> indexer.setGoal(Indexer.Goal.INTAKE),
                        () -> indexer.setGoal(Indexer.Goal.STOP),
                        indexer)))));

    // new InstantCommand(() -> feeder.);
  }
}
