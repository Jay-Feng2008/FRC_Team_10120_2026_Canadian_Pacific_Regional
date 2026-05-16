package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.AutoAlignTestCommand;
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

public class shootmiddle {
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
        AutoUtil.climbReleaseCommand(climb, flywheel),
        new InstantCommand(
            () ->
                drive.setPose(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? AutoUtil.GETatrtpose("middleaim") // 蓝方起始位置
                        : AllianceFlipUtil.apply(AutoUtil.GETatrtpose("middleaim")))),
        Commands.parallel(
                AutoUtil.followPath("middleaim"),
                // DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0),
                flywheel.runTrackTargetCommand(),
                flywheel_back.runTrackTargetCommand(),
                Commands.sequence(
                    Commands.waitSeconds(0.9),
                    Commands.parallel(
                        intakearm.intakeCommand(),
                        Commands.startEnd(
                            () -> feeder.setGoal(Feeder.Goal.INTAKE),
                            () -> feeder.setGoal(Feeder.Goal.STOP),
                            feeder),
                        Commands.startEnd(
                            () -> indexer.setGoal(Indexer.Goal.INTAKE),
                            () -> indexer.setGoal(Indexer.Goal.STOP),
                            indexer))))
            .withTimeout(5),

        /*----------------goto climb pose---------------------------*/
        AutoUtil.followPath("middleclimb"),
        Commands.either(
                new AutoAlignTestCommand(drive, ClimberConstants.CLIMB_RIGHT_FRONT_POSE, 0),
                new AutoAlignTestCommand(
                    drive, AllianceFlipUtil.apply(ClimberConstants.CLIMB_RIGHT_FRONT_POSE), 180),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            .withTimeout(2.0), // 强制给 2 秒对齐时间，方便观察
        /*----------------climb---------------------------*/
        new InstantCommand(
            () -> {
              climb.runVolts(4);
            }),
        Commands.waitSeconds(4),
        new InstantCommand(
            () -> {
              double ang = climb.getAngleRads();
              climb.runGoal(ang, 0.0, 0.0, 0.0);
            }));
  }
}
