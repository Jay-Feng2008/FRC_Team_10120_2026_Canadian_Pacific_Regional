package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.AutoAlignTestCommand2Hub;
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

public class StationIntakeDepot {
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
        new InstantCommand(
            () ->
                drive.setPose(
                    AllianceFlipUtil.apply(new Pose2d(4.001, 0.610, Rotation2d.fromDegrees(90))))),
        Commands.parallel(
            AutoUtil.climbReleaseCommand(climb, flywheel),
            AutoUtil.followPath("GotoStation"),
            Commands.runOnce(() -> intake.setGoal(Intake.Goal.OUTTAKE)),
            intakearm.intakeCommand()),
        Commands.waitSeconds(1),
        AutoUtil.followPath("AimHub"),
        Commands.parallel(
                new AutoAlignTestCommand2Hub(
                    drive, new Pose2d(2.692, 4.947, Rotation2d.fromDegrees(152.339)), true),
                flywheel.runTrackTargetCommand(),
                flywheel_back.runTrackTargetCommand(),
                Commands.sequence(
                    Commands.waitSeconds(0.7),
                    Commands.parallel(
                        Commands.startEnd(
                            () -> feeder.setGoal(Feeder.Goal.INTAKE),
                            () -> feeder.setGoal(Feeder.Goal.STOP),
                            feeder),
                        Commands.startEnd(
                            () -> indexer.setGoal(Indexer.Goal.INTAKE),
                            () -> indexer.setGoal(Indexer.Goal.STOP),
                            indexer),
                        new RepeatCommand(intakearm.runFixedCommand()))))
            .withTimeout(3),
        // Commands.parallel(
        //     flywheel.stopCommand(),
        //     flywheel_back.stopCommand(),
        //     new InstantCommand(() -> feeder.setGoal(Feeder.Goal.STOP)),
        //     new InstantCommand(() -> indexer.setGoal(Indexer.Goal.STOP))),
        Commands.parallel(
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.INTAKE)),
                intakearm.intakeCommand(),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)))
            .withTimeout(0.5),
        AutoUtil.followPath("postAimDepotIntake"),
        AutoUtil.followPath("depotaim"),
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
                        indexer),
                    new RepeatCommand(intakearm.runFixedCommand())))));
  }
}
