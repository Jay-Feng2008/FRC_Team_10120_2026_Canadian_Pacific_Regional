package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakearm.Intakearm;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.wheelback.Wheelback;

public class c1auto {
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
        // // new InstantCommand(
        // //     () ->
        // //         drive.setPose(
        // //             DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        // //                 ? new Pose2d(7.3, 5.8, Rotation2d.fromDegrees(180)) // 蓝方起始位置
        // //                 : new Pose2d(10.4, 2.15, Rotation2d.fromDegrees(0)) // 红方起始位置
        // //             )),
        // // AutoUtil.followChoreoPath("111"),
        // // new InstantCommand(() -> intakearm.runFixedCommand(2.2, 0.0)));
        // new SmashBumpCommand(drive),
        // Commands.race(
        //     AutoUtil.followPath("Down1"),
        //     Commands.parallel(
        //         Commands.startEnd(
        //             () -> intake.setGoal(Intake.Goal.INTAKE),
        //             () -> intake.setGoal(Intake.Goal.STOP),
        //             intake),
        //         Commands.startEnd(
        //             () -> indexer.setGoal(Indexer.Goal.INTAKE),
        //             () -> indexer.setGoal(Indexer.Goal.STOP),
        //             indexer))),
        // new SmashBumpCommand(drive),
        // Commands.parallel(
        //         flywheel.runTrackTargetCommand(),
        //         flywheel_back.runTrackTargetCommand(),
        //         Commands.sequence(
        //             Commands.waitSeconds(0.5),
        //             Commands.parallel(
        //                 Commands.startEnd(
        //                     () -> feeder.setGoal(Feeder.Goal.INTAKE),
        //                     () -> feeder.setGoal(Feeder.Goal.STOP),
        //                     feeder),
        //                 Commands.startEnd(
        //                     () -> indexer.setGoal(Indexer.Goal.INTAKE),
        //                     () -> indexer.setGoal(Indexer.Goal.STOP),
        //                     indexer))))
        //     .withTimeout(3),
        // AutoUtil.followPath("StationIntake"),
        // // addCommands(
        // //     Commands.parallel(
        // //             robotContainer.flywheel.runTrackTargetCommand(),
        // //             robotContainer.flywheel_back.runTrackTargetCommand(),
        // //             Commands.sequence(
        // //                 Commands.waitSeconds(0.5),
        // //                 Commands.parallel(
        // //                     Commands.startEnd(
        // //                         () -> robotContainer.feeder.setGoal(Feeder.Goal.INTAKE),
        // //                         () -> robotContainer.feeder.setGoal(Feeder.Goal.STOP),
        // //                         robotContainer.feeder),
        // //                     Commands.startEnd(
        // //                         () -> robotContainer.indexer.setGoal(Indexer.Goal.INTAKE),
        // //                         () -> robotContainer.indexer.setGoal(Indexer.Goal.STOP),
        // //                         robotContainer.indexer))))
        // //         .withTimeout(5));

        // new AutoAlignTestCommand(drive, new Pose2d(15.7, 5.1, Rotation2d.fromDegrees(0)), false)
        //     .withTimeout(5),
        // // AutoUtil.followPath("CenterClimb"),
        // new RepeatCommand(
        //         new InstantCommand(
        //             () -> {
        //               climb.runVolts(-4);
        //             }))
        //     .withTimeout(2),
        // new RepeatCommand(
        //     new InstantCommand(
        //         () -> {
        //           double ang = climb.getAngleRads();
        //           climb.runGoal(ang, 0.0, 0.0, 0.0);
        //         })));
        // new InstantCommand(
        //     () -> drive.setPose(new Pose2d(12.558, 6.274, Rotation2d.fromDegrees(152.339)))),
        // Commands.parallel(
        //     AutoUtil.climbReleaseCommand(climb, flywheel),
        //     Commands.sequence(
        //         new InstantCommand(
        //             () ->
        //                 drive.setPose(new Pose2d(12.558, 6.274,
        // Rotation2d.fromDegrees(152.339)))),
        //         AutoUtil.followPath("GotoStation"))),
        // Commands.waitSeconds(5),
        // AutoUtil.followPath("AimHub"),
        // Commands.parallel(
        //         DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0),
        //         flywheel.runTrackTargetCommand(),
        //         flywheel_back.runTrackTargetCommand(),
        //         Commands.sequence(
        //             Commands.waitSeconds(0.5),
        //             Commands.parallel(
        //                 Commands.startEnd(
        //                     () -> feeder.setGoal(Feeder.Goal.INTAKE),
        //                     () -> feeder.setGoal(Feeder.Goal.STOP),
        //                     feeder),
        //                 Commands.startEnd(
        //                     () -> indexer.setGoal(Indexer.Goal.INTAKE),
        //                     () -> indexer.setGoal(Indexer.Goal.STOP),
        //                     indexer))))
        //     .withTimeout(5),
        // Commands.parallel(
        //     new AutoAlignTestCommand2Hub(
        //         drive, new Pose2d(13.7, 2, Rotation2d.fromDegrees(0)), false),
        //     flywheel.runTrackTargetCommand(),
        //     flywheel_back.runTrackTargetCommand(),
        //     Commands.sequence(
        //         Commands.waitSeconds(0.5),
        //         Commands.parallel(
        //             Commands.startEnd(
        //                 () -> feeder.setGoal(Feeder.Goal.INTAKE),
        //                 () -> feeder.setGoal(Feeder.Goal.STOP),
        //                 feeder),
        //             Commands.startEnd(
        //                 () -> indexer.setGoal(Indexer.Goal.INTAKE),
        //                 () -> indexer.setGoal(Indexer.Goal.STOP),
        //                 indexer))))
        );
    // AutoAlignTestCommand2Hub(drive, new Pose2d(13.7, 2, Rotation2d.fromDegrees(0)), false);

    // new InstantCommand(() -> feeder.);
  }
}
