package frc.robot.autos.CenterField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoAlignTestCommand2Hub;
import frc.robot.commands.SmashBumpCommand;
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

public class CenterFieldCollectScoreMirrored {
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
                    AllianceFlipUtil.apply(new Pose2d(3.550, 5.864, Rotation2d.fromDegrees(-90))))),
        Commands.parallel(
            new SmashBumpCommand(drive), AutoUtil.climbReleaseCommand(climb, flywheel)),
        Commands.parallel(
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.INTAKE)),
                intakearm.intakeCommand(),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)))
            .withTimeout(0.5),
        AutoUtil.followPath("TraverseHerdMirrored"),
        new SmashBumpCommand(drive),
        Commands.parallel(
            Commands.either(
                new AutoAlignTestCommand2Hub(
                    drive, new Pose2d(14.044, 5.601, Rotation2d.fromDegrees(36.631)), false),
                new AutoAlignTestCommand2Hub(
                    drive, new Pose2d(2.533, 2.761, Rotation2d.fromDegrees(31.051)), false),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red),
            flywheel.runTrackTargetCommand(),
            flywheel_back.runTrackTargetCommand(),
            Commands.sequence(
                Commands.waitSeconds(1.5),
                Commands.parallel(
                    Commands.startEnd(
                        () -> feeder.setGoal(Feeder.Goal.INTAKE),
                        () -> feeder.setGoal(Feeder.Goal.STOP),
                        feeder),
                    Commands.startEnd(
                        () -> indexer.setGoal(Indexer.Goal.INTAKE),
                        () -> indexer.setGoal(Indexer.Goal.STOP),
                        indexer)))));
  }
}
