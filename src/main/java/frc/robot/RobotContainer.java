// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.autos.CenterField.CFCSClimb;
import frc.robot.autos.CenterField.CFCSMirroredClimb;
import frc.robot.autos.CenterField.CenterFieldCollectScore;
import frc.robot.autos.CenterField.CenterFieldCollectScoreMirrored;
import frc.robot.autos.DownMagic;
import frc.robot.autos.StationIntakeClimb;
import frc.robot.autos.StationIntakeDepot;
import frc.robot.autos.c1auto;
import frc.robot.autos.depot;
import frc.robot.autos.outpost;
import frc.robot.autos.shootmiddle;
import frc.robot.commands.AutoAlignTestCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FlywheelWarmupLEDCommand;
import frc.robot.commands.SmashBumpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShotDataCollector;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOKrakenFOC;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakearm.Intakearm;
import frc.robot.subsystems.intakearm.IntakearmIOReal;
import frc.robot.subsystems.intakearm.IntakearmIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.rollers.RollerSystemIOReal;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wheelback.Wheelback;
import frc.robot.subsystems.wheelback.WheelbackIOReal;
import frc.robot.subsystems.wheelback.WheelbackIOSim;
import frc.robot.util.controller.TriggerUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {

  // Drivers
  private final Driver1 driver1 = new Driver1();
  private final Driver2 driver2 = new Driver2();
  //   private final CommandPS5Controller cPs5Controller = new CommandPS5Controller(2);
  private double controller_deadband = Constants.ControllerDeadband;
  private double drive_factor = 1.0;
  private boolean intaking_FLAG = false;

  // Subsystems
  public final Drive drive;
  public final Intake intake;
  public final Feeder feeder;
  public final Flywheel flywheel;
  public final Wheelback flywheel_back;
  public final Indexer indexer;
  public final Intakearm intake_arm;
  // public final Hood hood;
  // public final UpperStructure upperStructure;
  public final Vision vision;
  //   public final Hanger hanger;
  public final ClimbIO climbIO;
  public final Climb climb;
  public final LED led;

  private final TalonFX Motor_FLD = new TalonFX(19, "canivore");
  private final TalonFX Motor_FLS = new TalonFX(4, "canivore");
  private final TalonFX Motor_FRD = new TalonFX(2, "canivore");
  private final TalonFX Motor_FRS = new TalonFX(3, "canivore");
  private final TalonFX Motor_BLD = new TalonFX(5, "canivore");
  private final TalonFX Motor_BLS = new TalonFX(22, "canivore");
  private final TalonFX Motor_BRD = new TalonFX(8, "canivore");
  private final TalonFX Motor_BRS = new TalonFX(46, "canivore");

  private final TalonFX Motor_Intake = new TalonFX(7, "canivore");
  private final TalonFX Motor_IntakeArm = new TalonFX(10, "canivore");
  private final TalonFX Motor_Index = new TalonFX(11, "canivore");
  private final TalonFX Motor_Feeder = new TalonFX(12, "rio");
  private final TalonFX Motor_Climb = new TalonFX(16, "rio");
  private final TalonFX Motor_FlywheelL = new TalonFX(15, "rio");
  private final TalonFX Motor_FlywheelR = new TalonFX(1, "rio");
  private final TalonFX Motor_FlywheelBack = new TalonFX(6, "rio");

  private final Orchestra orchestra = new Orchestra();
  private final String[] songs = new String[] {"HZ.chrp", "winter.chrp"};
  private int song_counter = 0;

  private final Alert controllerDisconnected =
      new Alert("controller disconnected (port 0).", AlertType.kWarning);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    orchestra.loadMusic(songs[0]);

    orchestra.addInstrument(Motor_FLD, 0);
    orchestra.addInstrument(Motor_FRD, 0);
    orchestra.addInstrument(Motor_Intake, 0);
    orchestra.addInstrument(Motor_IntakeArm, 0);
    orchestra.addInstrument(Motor_Index, 0);
    orchestra.addInstrument(Motor_Feeder, 0);
    orchestra.addInstrument(Motor_Climb, 0);
    orchestra.addInstrument(Motor_FlywheelL, 0);
    orchestra.addInstrument(Motor_FlywheelR, 0);
    orchestra.addInstrument(Motor_FlywheelBack, 0);

    orchestra.addInstrument(Motor_FLS, 1);
    orchestra.addInstrument(Motor_FRS, 1);
    orchestra.addInstrument(Motor_BLS, 2);
    orchestra.addInstrument(Motor_BRS, 2);
    orchestra.addInstrument(Motor_BLD, 3);
    orchestra.addInstrument(Motor_BRD, 3);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::getGyroRateDegPerSec,
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        intake = new Intake(new RollerSystemIOReal(7, true, 1));
        indexer = new Indexer(new RollerSystemIOReal(11, false, 1));
        feeder = new Feeder(new RollerSystemIOReal(12, false, 1.1));
        intake_arm = new Intakearm(new IntakearmIOReal());
        flywheel = new Flywheel(new FlywheelIOReal(15, false));
        flywheel_back = new Wheelback(new WheelbackIOReal(6, false));
        // hanger = new Hanger(new HangerIOReal(Ports.kHanger, true));
        climbIO = new ClimbIOKrakenFOC();
        climb = new Climb(climbIO);
        led = new LED(drive);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::getGyroRateDegPerSec,
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intake = new Intake(new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1));
        intake_arm = new Intakearm(new IntakearmIOSim());
        indexer = new Indexer(new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1));
        feeder = new Feeder(new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1));
        flywheel = new Flywheel(new FlywheelIOSim());
        flywheel_back = new Wheelback(new WheelbackIOSim());
        // hanger = new Hanger(new HangerIOReal(Ports.kHanger, true));
        climbIO = new ClimbIOKrakenFOC();
        climb = new Climb(climbIO);
        led = new LED(drive);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::getGyroRateDegPerSec,
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {});
        intake = new Intake(new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1));
        intake_arm = new Intakearm(new IntakearmIOSim());
        feeder = new Feeder(new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1));
        indexer = new Indexer(new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1, 1));
        flywheel = new Flywheel(new FlywheelIOSim());
        flywheel_back = new Wheelback(new WheelbackIOSim());
        // hanger = new Hanger(new HangerIOReal(Ports.kHanger, true));
        climbIO = new ClimbIOKrakenFOC();
        climb = new Climb(climbIO);
        led = new LED(drive);
        break;
    }

    ShotCalculator.getInstance().init(drive);

    // NamedCommands.registerCommand(
    //     "INTAKE",
    //     Commands.parallel(
    //         Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE), intake)
    //         // Commands.runOnce(() -> intake_arm.setGoal(Intakearm.Goal.INTAKE), intake_arm))
    //         ));
    // NamedCommands.registerCommand(
    //     "OUTTAKE", Commands.runOnce(() -> intake.setGoal(Intake.Goal.OUTTAKE), intake));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "1coral_middle",
        c1auto.build(drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "outpost",
        outpost.build(drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "depot",
        depot.build(drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "shootmiddle",
        shootmiddle.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "StationIntakeClimb",
        StationIntakeClimb.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "StationIntakeDepot",
        StationIntakeDepot.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "CenterFieldCollectScore",
        CenterFieldCollectScore.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "CenterFieldCollectScoreMirrored",
        CenterFieldCollectScoreMirrored.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "CFCSMirroredClimb",
        CFCSMirroredClimb.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption(
        "CFCSClimb",
        CFCSClimb.build(
            drive, intake, flywheel, flywheel_back, indexer, feeder, climb, intake_arm));
    autoChooser.addOption("Down", new DownMagic(this).withTimeout(20.5));
    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption(
    //     "pathfinding test",
    //     Commands.sequence(
    //         drive.pathfindToTargetPose(
    //             AllianceFlipUtil.apply(
    //                 new Pose2d(
    //                     FieldConstants.Hub.nearFace.getTranslation(),
    //                     Rotation2d.fromDegrees(0))))));
    // // autoChooser.addOption("test2026_123", autoRoutines.test2026_123());
    // autoChooser.addDefaultOption("Example Auto", AutoBuilder.buildAuto("Example Auto"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -MathUtil.applyDeadband(
                        Driver1.leftController.getRawAxis(driver1.translationAxis),
                        controller_deadband)
                    * drive_factor,
            () ->
                -MathUtil.applyDeadband(
                        Driver1.leftController.getRawAxis(driver1.strafeAxis), controller_deadband)
                    * drive_factor,
            () ->
                -MathUtil.applyDeadband(
                    Driver1.rightController.getRawAxis(driver1.rotationAxis),
                    controller_deadband)));

    // Lock to 0° when A button is held
    Driver1.HeadZero.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () ->
                -MathUtil.applyDeadband(
                        Driver1.leftController.getRawAxis(driver1.translationAxis),
                        controller_deadband)
                    * drive_factor,
            () ->
                -MathUtil.applyDeadband(
                        Driver1.leftController.getRawAxis(driver1.strafeAxis), controller_deadband)
                    * drive_factor,
            () -> Rotation2d.kZero));

    // Reset gyro to 0° when button is pressed 自动开始时设置为路径起始点 也许不需要 可以在开始路径之前重置
    Driver1.zeroHeading.onTrue(
        Commands.runOnce(
                () -> {
                  if (DriverStation.isDisabled()) {
                    // if (autoChooser.getSendableChooser().getSelected() == "Up") {
                    //   drive.setPose(UpOut.getStartPose(DriverStation.getAlliance().get()));
                    // } else {
                    //   drive.setPose(DownMagic.getStartPose(DriverStation.getAlliance().get()));
                    // }
                  } else {
                    drive.setPose(
                        DriverStation.getAlliance().get() == Alliance.Blue
                            ? new Pose2d(
                                drive.getPose().getTranslation(), Rotation2d.fromDegrees(0))
                            : new Pose2d(
                                drive.getPose().getTranslation(), Rotation2d.fromDegrees(180)));
                  }
                },
                drive)
            .ignoringDisable(true));

    Driver1.IntakeArmHome.whileTrue(intake_arm.homingCommand());

    Driver1.PlayMusic1.and(Driver1.AimSwitch)
        .and(Driver1.PlayMusic3)
        .and(Driver1.PlayMusic4)
        .and(Driver1.PlayMusic5)
        .and(Driver1.PlayMusic6)
        .onTrue(Commands.runOnce(() -> orchestra.play()));
    Driver1.StopMusic.onTrue(Commands.runOnce(() -> orchestra.stop()));

    Driver1.NextSong.toggleOnTrue(
            Commands.startEnd(
                () -> song_counter += 1,
                () -> orchestra.loadMusic(songs[song_counter % songs.length])))
        .whileTrue(
            new InstantCommand(
                () -> {
                  System.out.println(songs[song_counter % songs.length]);
                }));

    Driver2.AimHub.and(Driver1.PlayMusic1.negate())
        .or(Driver1.Driver1AimHub)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () ->
                    -MathUtil.applyDeadband(
                            Driver1.leftController.getRawAxis(driver1.translationAxis),
                            controller_deadband)
                        * drive_factor,
                () ->
                    -MathUtil.applyDeadband(
                            Driver1.leftController.getRawAxis(driver1.strafeAxis),
                            controller_deadband)
                        * drive_factor));

    Driver1.RecordData.onTrue(
        Commands.runOnce(
            () ->
                ShotDataCollector.record(
                    ShotCalculator.getInstance().getParameters().lookaheadTurretToTargetDistance(),
                    flywheel.getGoalVelocity(),
                    flywheel_back.getGoalVelocity(),
                    flywheel.getTOF())));
    Driver1.ReportData.onTrue(Commands.runOnce(() -> ShotDataCollector.dumpToConsole()));

    // controller
    //     .rightBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               // drive.setPose(
    //               //     new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //               upperStructure.setState(
    //                   UpperStructureState.TestTrack); // TODO: 机器位置控制 需要在某个地方重写 可以参考10711

    //               // intake.setState(IntakeState.Stop);
    //             }));

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         flywheel
    //             .runTrackTargetCommand()
    //             .alongWith(hood.runTrackTargetCommand())
    //             .alongWith(turretFL.aim_Hub_Command()));
    // .toggleOnFalse(Commands.parallel(flywheel.stopCommand()));
    // 有头模式
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         DriveCommands.joystickDriveRobotRelative(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> -controller.getRightX()));
    // controller
    //     .rightStick()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> drive.setPose(new Pose2d(new Translation2d(3, 5), Rotation2d.kZero)),
    //             drive));

    // controller
    //     .povDown()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> drive.setPose(new Pose2d(new Translation2d(7, 4), Rotation2d.kZero)),
    //             drive));

    // controller.a().whileTrue(flywheel.runTestVelocityCommand());

    Driver1.LiftIntakeArm.or(Driver2.LiftIntakeArm.and(Driver1.PlayMusic1.negate()))
        .whileTrue(new RepeatCommand(intake_arm.runFixedCommand()))
        .toggleOnFalse(intake_arm.runFixedCommand());

    Driver2.CoastIntakeArm.and(Driver1.PlayMusic1.negate()).whileTrue(intake_arm.coastCommand());

    Driver1.RevertShooter
        // .negate()
        .whileTrue(
        Commands.parallel(
            Commands.startEnd(
                () -> feeder.setGoal(Feeder.Goal.OUTTAKE),
                () -> feeder.setGoal(Feeder.Goal.STOP),
                feeder),
            Commands.startEnd(
                () -> indexer.setGoal(Indexer.Goal.OUTTAKE),
                () -> indexer.setGoal(Indexer.Goal.STOP),
                indexer)));

    Driver1.RightTrigger
        // .negate()
        .whileTrue(
            Commands.parallel(
                flywheel.runTrackTargetCommand(),
                flywheel_back.runTrackTargetCommand(),
                new FlywheelWarmupLEDCommand(led)))
        .and(() -> ShotCalculator.getInstance().getParameters().isValid())
        .and(flywheel::atGoal)
        .whileTrueContinuous(
            Commands.sequence(
                new WaitCommand(0.3),
                Commands.parallel(
                    Commands.startEnd(
                        () -> feeder.setGoal(Feeder.Goal.INTAKE),
                        () -> feeder.setGoal(Feeder.Goal.STOP),
                        feeder),
                    Commands.startEnd(
                        () -> indexer.setGoal(Indexer.Goal.INTAKE),
                        () -> indexer.setGoal(Indexer.Goal.STOP),
                        indexer))));

    // Driver1.LeftTrigger.whileTrue(
    //     Commands.parallel(
    //         Commands.runOnce(() -> intake.switchState()), intake_arm.intakeCommand()));
    Driver1.LeftTrigger.whileTrue(
            Commands.parallel(
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                intake_arm.intakeCommand()))
        .toggleOnFalse(Commands.runOnce(() -> intake.setGoal(Intake.Goal.STOP)));

    Driver1.IntakeArmPark.whileTrue(intake_arm.parkCommand());
    // .onFalse(intake_arm.parkCommand());
    // .negate()
    // .whileTrue(
    //     Commands.parallel(
    //         Commands.startEnd(
    //             () -> intake.setGoal(Intake.Goal.INTAKE),
    //             () -> intake.setGoal(Intake.Goal.STOP)
    //             intake),
    //         Commands.startEnd(
    //             () -> indexer.setGoal(Indexer.Goal.INTAKE),
    //             () -> indexer.setGoal(Indexer.Goal.STOP),
    //             indexer)));

    // Driver1.Warmup.whileTrue(flywheel.runTestVelocityCommand());
    // Driver1.Warmup.whileTrue(
    //     Commands.parallel(flywheel.runTrackTargetCommand(),
    // flywheel_back.runTrackTargetCommand()));
    Driver1.Warmup.whileTrue(
        Commands.parallel(flywheel.runFixedCommand(), flywheel_back.runFixedCommand()));

    Driver1.BackShooting.whileTrue(
        Commands.parallel(flywheel.runFixedCommand(200), flywheel_back.runFixedCommand(450)));

    Driver2.RightTrigger.and(Driver1.PlayMusic1.negate())
        .whileTrue(
            Commands.parallel(
                flywheel.runTrackTargetCommand(), flywheel_back.runTrackTargetCommand()));

    // Climb
    Driver1.Climb.onTrue(
            new InstantCommand(
                () -> {
                  led.showLED(LEDState.Climb);
                  climb.runVolts(-4);
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  double ang = climb.getAngleRads();
                  climb.runGoal(ang, 0.0, 0.0, 0.0);
                }));

    Driver1.ClimbDown.onTrue(
            new InstantCommand(
                () -> {
                  climb.runVolts(4);
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  double ang = climb.getAngleRads();
                  climb.runGoal(ang, 0.0, 0.0, 0.0);
                }));

    Driver2.Climb.and(Driver1.PlayMusic1.negate())
        .onTrue(
            new InstantCommand(
                () -> {
                  climb.runVolts(-4);
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  double ang = climb.getAngleRads();
                  climb.runGoal(ang, 0.0, 0.0, 0.0);
                }));

    Driver2.ClimbDown.and(Driver1.PlayMusic1.negate())
        .onTrue(
            new InstantCommand(
                () -> {
                  climb.runVolts(4);
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  double ang = climb.getAngleRads();
                  climb.runGoal(ang, 0.0, 0.0, 0.0);
                }));

    // Driver1.TestClimbAlign.whileTrue(
    //         new AutoAlignTestCommand(
    //             drive, new Pose2d(15.7, 5.1, Rotation2d.fromDegrees(180)), -120))
    //     .toggleOnFalse(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () ->
    //                 -MathUtil.applyDeadband(
    //                         Driver1.leftController.getRawAxis(driver1.translationAxis),
    //                         controller_deadband)
    //                     * drive_factor,
    //             () ->
    //                 -MathUtil.applyDeadband(
    //                         Driver1.leftController.getRawAxis(driver1.strafeAxis),
    //                         controller_deadband)
    //                     * drive_factor,
    //             () ->
    //                 -MathUtil.applyDeadband(
    //                     Driver1.rightController.getRawAxis(driver1.rotationAxis),
    //                     controller_deadband)));

    Driver1.TestClimbAlign.whileTrue(
            Commands.sequence(
                new AutoAlignTestCommand(
                    // drive, new Pose2d(1.178, 4.643, Rotation2d.fromDegrees(0.000)), -90),
                    drive,
                    new Pose2d(15.36375, 3.3707000000000003, Rotation2d.fromDegrees(0.0)),
                    90),
                new RepeatCommand(
                        DriveCommands.joystickDrive(drive, () -> 0.0, () -> -0.20, () -> 0))
                    .withTimeout(0.5),
                new RepeatCommand(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0))
                    .withTimeout(0.01),
                new RepeatCommand(
                        DriveCommands.joystickDrive(drive, () -> -0.50, () -> 0.0, () -> 0))
                    .withTimeout(0.5),
                new RepeatCommand(DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 0))
                    .withTimeout(0.01)))
        .toggleOnFalse(
            DriveCommands.joystickDrive(
                drive,
                () ->
                    -MathUtil.applyDeadband(
                            Driver1.leftController.getRawAxis(driver1.translationAxis),
                            controller_deadband)
                        * drive_factor,
                () ->
                    -MathUtil.applyDeadband(
                            Driver1.leftController.getRawAxis(driver1.strafeAxis),
                            controller_deadband)
                        * drive_factor,
                () ->
                    -MathUtil.applyDeadband(
                        Driver1.rightController.getRawAxis(driver1.rotationAxis),
                        controller_deadband)));

    Driver1.SmashBumpLeft.or(Driver1.SmashBumpRight).whileTrue(new SmashBumpCommand(drive));
    /*---------------controller2---------------------------*/
    // cPs5Controller.triangle().whileTrue(Commands.runOnce(() -> climb.runVolts(4)));
    // cPs5Controller.cross().whileTrue(Commands.runOnce(() -> climb.runVolts(-4)));
    // cPs5Controller.circle().whileTrue(Commands.runOnce(() -> climb.resetangle()));
    // controller
    //     .leftTrigger()
    //     .whileTrue(
    //         DriveCommands.joystickDriveFacingPoint(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () ->
    //                 AllianceFlipUtil.apply(
    //                     FieldConstants.Hub.topCenterPoint.toTranslation2d())));
    // controller
    //     .a()
    //     .whileTrue(flywheel.runTrackTargetCommand())
    //     .and(() -> ShotCalculator.getInstance().getParameters().isValid())
    //     .and(flywheel::atGoal)
    //     // .whileTrueContinuous(
    //     //     // .whileTrueContinuous(...) is not a WPILib command,
    //     //     // use whileTrue instead for code integrity.
    //     //     // TODO: confirm difference between whileTrueContinuous and whileTrue
    //     //     Commands.parallel(
    //     //         Commands.startEnd(
    //     //             () -> feeder.setGoal(Feeder.Goal.SHOOT),
    //     //             () -> feeder.setGoal(Feeder.Goal.STOP),
    //     //             feeder),
    //     //         Commands.startEnd(
    //     //             () -> indexer.setGoal(Indexer.Goal.SHOOT),
    //     //             () -> indexer.setGoal(Indexer.Goal.STOP),
    //     //             indexer)))
    //     .whileTrue(
    //         Commands.parallel(
    //             Commands.startEnd(
    //                 () -> feeder.setGoal(Feeder.Goal.SHOOT),
    //                 () -> feeder.setGoal(Feeder.Goal.STOP),
    //                 feeder),
    //             Commands.startEnd(
    //                 () -> indexer.setGoal(Indexer.Goal.SHOOT),
    //                 () -> indexer.setGoal(Indexer.Goal.STOP),
    //                 indexer)))
    //     .onFalse(
    //         Commands.startEnd(
    //                 () -> feeder.setGoal(Feeder.Goal.OUTTAKE),
    //                 () -> feeder.setGoal(Feeder.Goal.STOP),
    //                 feeder)
    //             .withTimeout(0.5));

    // SIM:
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                     SimDriver.controller.getRawAxis(SimDriver.translationAxis),
    //                     controller_deadband)
    //                 * drive_factor,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                     SimDriver.controller.getRawAxis(SimDriver.strafeAxis),
    // controller_deadband)
    //                 * drive_factor,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                 SimDriver.controller.getRawAxis(SimDriver.rotationAxis),
    // controller_deadband)));

    // // Lock to 0° when A button is held
    // SimDriver.HeadZero.whileTrue(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                     SimDriver.controller.getRawAxis(SimDriver.translationAxis),
    //                     controller_deadband)
    //                 * drive_factor,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                     SimDriver.controller.getRawAxis(SimDriver.strafeAxis),
    // controller_deadband)
    //                 * drive_factor,
    //         () -> Rotation2d.kZero));

    // // Reset gyro to 0° when button is pressed
    // SimDriver.zeroHeading.onTrue(
    //     Commands.runOnce(
    //             () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
    // Rotation2d.kZero)),
    //             drive)
    //         .ignoringDisable(true));
    // // Auto aim command example

    // SimDriver.AimTarget.whileTrue(
    //     DriveCommands.joystickDriveFacingPoint(
    //         drive,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                     Driver1.leftController.getRawAxis(driver1.translationAxis),
    //                     controller_deadband)
    //                 * drive_factor,
    //         () ->
    //             -MathUtil.applyDeadband(
    //                     Driver1.leftController.getRawAxis(driver1.strafeAxis),
    // controller_deadband)
    //                 * drive_factor,
    //         () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())));

    // SimDriver.Warmup.whileTrue(flywheel.runTestVelocityCommand());

    // SimDriver // 吐球 有最高优先级
    //     .RightTrigger.whileTrue(
    //     Commands.startEnd(
    //             () -> feeder.setGoal(Feeder.Goal.SHOOT),
    //             () -> feeder.setGoal(Feeder.Goal.STOP),
    //             feeder)
    //         .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); // 任何想抢占这些硬件的后续指令都会被拒绝
    // SimDriver.LeftTrigger
    //     // .negate()
    //     .whileTrue(
    //     Commands.parallel(
    //         Commands.startEnd(
    //             () -> intake.setGoal(Intake.Goal.INTAKE),
    //             () -> intake.setGoal(Intake.Goal.STOP),
    //             intake),
    //         Commands.startEnd(
    //             () -> indexer.setGoal(Indexer.Goal.INTAKE),
    //             () -> indexer.setGoal(Indexer.Goal.STOP),
    //             indexer)));

    // // Climb
    // SimDriver.ClimbUp.onTrue(
    //         new InstantCommand(
    //             () -> {
    //               climb.runVolts(-4);
    //             }))
    //     .toggleOnFalse(
    //         new InstantCommand(
    //             () -> {
    //               double ang = climb.getAngleRads();
    //               climb.runGoal(ang, 0.0, 0.0, 0.0);
    //             }));

    // SimDriver.ClimbDown.onTrue(
    //         new InstantCommand(
    //             () -> {
    //               climb.runVolts(4);
    //             }))
    //     .toggleOnFalse(
    //         new InstantCommand(
    //             () -> {
    //               double ang = climb.getAngleRads();
    //               climb.runGoal(ang, 0.0, 0.0, 0.0);
    //             }));
  }

  /** Update dashboard outputs. */
  //   public void updateDashboardOutputs() {
  //     // Publish match time
  //     SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

  //     SmartDashboard.putNumber("Batter Voltage", RobotController.getBatteryVoltage());
  //     // Controller disconnected alerts
  //
  // controllerDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));

  //     //
  // secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
  //     // overrideDisconnected.set(!overrides.isConnected());
  //   }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return FieldConstants.defaultAprilTagType;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
