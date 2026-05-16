package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignTestCommand;
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

public class StationIntakeClimb {
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
            Commands.sequence(
                new InstantCommand(
                    () ->
                        drive.setPose(
                            AllianceFlipUtil.apply(
                                new Pose2d(4.001, 0.610, Rotation2d.fromDegrees(90))))),
                AutoUtil.followPath("GotoStation"),
                Commands.waitSeconds(1),
                AutoUtil.followPath("AimHub")),
            flywheel.runFixedCommand().withTimeout(1), // Release Container Limitor
            Commands.sequence(
                new RepeatCommand(
                        new InstantCommand(
                            () -> {
                              climb.runVolts(4);
                            }))
                    .withTimeout(4.3),
                new RepeatCommand(
                        new InstantCommand(
                            () -> {
                              double ang = climb.getAngleRads();
                              climb.runGoal(ang, 0.0, 0.0, 0.0);
                            }))
                    .withTimeout(0.1)),
            Commands.sequence(
                new WaitCommand(0.5),
                Commands.parallel(
                    Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                    intakearm.intakeCommand())
                // new WaitCommand(1.0),
                // intakearm.runFixedCommand()
                )),
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
                        new RepeatCommand(intakearm.runFixedCommand()))))
            .withTimeout(3),
        AutoUtil.followPath("ApproachTower"),
        Commands.either(
                new AutoAlignTestCommand(
                    drive, new Pose2d(15.59475, 5.1627, Rotation2d.fromDegrees(180)), -90),
                new AutoAlignTestCommand(
                    drive, new Pose2d(0.947, 2.851, Rotation2d.fromDegrees(0)), 90),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            .withTimeout(10),
        new RepeatCommand(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.20, () -> 0))
            .withTimeout(0.5),
        new RepeatCommand(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0))
            .withTimeout(0.01),
        new RepeatCommand(DriveCommands.joystickDrive(drive, () -> 0.50, () -> 0.0, () -> 0))
            .withTimeout(0.5),
        new RepeatCommand(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0))
            .withTimeout(0.01),
        new RepeatCommand(
                new InstantCommand(
                    () -> {
                      climb.runVolts(-4);
                    }))
            .withTimeout(4.3),
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  double ang = climb.getAngleRads();
                  climb.runGoal(ang, 0.0, 0.0, 0.0);
                })));

    // AutoAlignTestCommand2Hub(drive, new Pose2d(13.7, 2, Rotation2d.fromDegrees(0)), false);

    // new InstantCommand(() -> feeder.);
  }
}
