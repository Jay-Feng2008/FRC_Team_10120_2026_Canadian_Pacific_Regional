// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakearm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakearm.IntakearmIO.IntakearmIOOutputMode;
import frc.robot.subsystems.intakearm.IntakearmIO.IntakearmIOOutputs;
// import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
// import frc.robot.AlphaMechanism3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intakearm extends SubsystemBase {
  private final IntakearmIO io;
  private final IntakearmIOInputsAutoLogged inputs = new IntakearmIOInputsAutoLogged();
  private final IntakearmIOOutputs outputs = new IntakearmIOOutputs();

  // private static final double minAngle = Units.degreesToRadians(19);
  // private static final double maxAngle = Units.degreesToRadians(51);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intakearm/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intakearm/kD");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intakearm/kI");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intakearm/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intakearm/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intakearm/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intakearm/kG");
  private static final LoggedTunableNumber angle = new LoggedTunableNumber("Intakearm/Angle", 2.2);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Intakearm/ToleranceDeg");

  static {
    // 真实虚拟机调试参数不同
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceDeg.initDefault(1.0);
    } else {
      kP.initDefault(IntakearmConstants.kP);
      kI.initDefault(IntakearmConstants.kI);
      kD.initDefault(IntakearmConstants.kD);
      kS.initDefault(IntakearmConstants.kS);
      kV.initDefault(IntakearmConstants.kV);
      kA.initDefault(IntakearmConstants.kA);
      kG.initDefault(IntakearmConstants.kG);
      toleranceDeg.initDefault(IntakearmConstants.IntakearmtoleranceDeg);
    }
  }

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(1.0, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Intakearm motor disconnected!", Alert.AlertType.kWarning);

  // @Setter private BooleanSupplier coastOverride = () -> false;

  private double goalAngle = IntakearmConstants.IntakearmInitialAngle;
  private double goalVelocity = 0.0;

  // 归零
  private boolean IntakearmZeroed = true;

  public Intakearm(IntakearmIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }
  //
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intakearm", inputs);
    Logger.recordOutput("Intake/Mode", outputs.mode);

    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = IntakearmIOOutputMode.COAST;
      // if (coastOverride.getAsBoolean()) {
      //   outputs.mode = IntakearmIOOutputMode.COAST;
      // }
    }

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    }
    // Visualize turret in 3D
    // AlphaMechanism3d.getMeasured().setIntakearmAngle(new Rotation2d(getMeasuredAngleRad()));

    // Record cycle time
    // LoggedTracer.record("Intakearm");
    if (DriverStation.isEnabled()) {
      outputs.positionRad =
          MathUtil.clamp(
              goalAngle,
              IntakearmConstants.IntakearmMinAngle,
              IntakearmConstants.IntakearmMaxAngle);
      outputs.velocityRadsPerSec = goalVelocity;
      // outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;
      // TODO:添加堵转限制 电流超过4A则进入COAST模式 需要测试
      if (Math.abs(inputs.torqueCurrentAmps) > 45) {
        outputs.mode = IntakearmIOOutputMode.COAST;
      }
      // Log state
      Logger.recordOutput("Intakearm/Profile/GoalPositionRad", goalAngle);
      Logger.recordOutput("Intakearm/Profile/GoalVelocityRadPerSec", goalVelocity);
    }
    Logger.recordOutput("Intakearm/Profile/mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  // @Override
  // public void periodicAfterScheduler() {
  //   if (DriverStation.isEnabled() && IntakearmZeroed) {
  //     outputs.positionRad = MathUtil.clamp(goalAngle, minAngle, maxAngle) - IntakearmOffset;
  //     outputs.velocityRadsPerSec = goalVelocity;
  //     outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;

  //     // Log state
  //     Logger.recordOutput("Intakearm/Profile/GoalPositionRad", goalAngle);
  //     Logger.recordOutput("Intakearm/Profile/GoalVelocityRadPerSec", goalVelocity);
  //   }

  //   io.applyOutputs(outputs);
  // }

  private void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
    // outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;
  }

  @AutoLogOutput(key = "Intakearm/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleRad() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  private void zero() {
    IntakearmZeroed = true;
    io.resetAngle(0.0);
  }

  private void coast() {
    outputs.mode = IntakearmIOOutputMode.COAST;
  }

  public Command intakeCommand() {
    return runOnce(
        () -> {
          outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;
          setGoalParams(2.2, 0.0);
        });
  }

  public Command parkCommand() {
    return runOnce(
        () -> {
          outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;
          setGoalParams(0.0, 0.0);
        });
  }

  // public Command runTrackTargetCommand() {
  //   return run(
  //       () -> {
  //         var params = ShotCalculator.getInstance().getParameters();
  //         setGoalParams(params.IntakearmAngle(), params.IntakearmVelocity());
  //       });
  // }

  public Command runFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
    return run(() -> setGoalParams(angle.getAsDouble(), velocity.getAsDouble()));
  }

  public Command runFixedCommand(double angle, double velocity) {
    return run(() -> setGoalParams(angle, velocity));
  }

  public Command runFixedCommand() {
    // return runOnce(
    //     () -> {
    //       outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;
    //       setGoalParams(angle.get(), 0.0);
    //     });
    // return startEnd(() -> setGoalParams(0, 0.0), () -> setGoalParams(angle.get(), 0.0))
    //     .withTimeout(5);

    return new InstantCommand(() -> outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP)
        .andThen(new InstantCommand(() -> setGoalParams(0.7, 0.0)))
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(() -> setGoalParams(angle.get(), 0.0)))
        .andThen(new WaitCommand(0.5));
  }

  public Command homingCommand() {
    return Commands.sequence(
        runOnce(
            () -> {
              outputs.mode = IntakearmIOOutputMode.HOMING;
              // io.applyOutputs(outputs);
            }),
        Commands.waitUntil(() -> inputs.torqueCurrentAmps > 4), // 修改为扭矩电流 TODO：需要测试
        runOnce(
            () -> {
              io.resetAngle(2.2); // Set Postion to 0.0
              outputs.mode = IntakearmIOOutputMode.COAST;
              // io.applyOutputs(outputs);
            }));
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }

  public Command coastCommand() {
    return runOnce(this::coast).ignoringDisable(true);
  }
}
