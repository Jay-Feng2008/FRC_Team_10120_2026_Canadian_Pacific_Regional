// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert followerDisconnected;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG");
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Flywheel/VelocityTolerance", 10.0); // rad/s
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);
  private static final LoggedTunableNumber fixspeed =
      new LoggedTunableNumber("Flywheel/fixtargetspeed", 167);
  private static final LoggedTunableNumber TOF = new LoggedTunableNumber("Flywheel/TOF", 0);

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
      velocityTolerance.initDefault(1.0);
    } else {
      kP.initDefault(FlywheelConstants.kP);
      kI.initDefault(FlywheelConstants.kI);
      kD.initDefault(FlywheelConstants.kD);
      kS.initDefault(FlywheelConstants.kS);
      kV.initDefault(FlywheelConstants.kV);
      kA.initDefault(FlywheelConstants.kA);
      kG.initDefault(FlywheelConstants.kG);
      velocityTolerance.initDefault(FlywheelConstants.velocityTolerance);
    }
  }

  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  @AutoLogOutput private long shotCount = 0;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
    followerDisconnected =
        new Alert("Flywheel follower motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // 更新 Tunable Numbers
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
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

    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    followerDisconnected.set(!motorFollowerConnectedDebouncer.calculate(inputs.followerConnected));

    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRadsPerSec);
    io.applyOutputs(outputs);
  }

  // @Override
  // public void periodicAfterScheduler() {
  //   Logger.recordOutput("Flywheel/Mode", outputs.mode);
  //   io.applyOutputs(outputs);
  // }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRadsPerSec) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    outputs.volts = 0.0;
    // 2. 计算是否到达目标 (At Goal Logic)
    // 只有当设定值非0且误差在容差范围内时，才认为到达目标
    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec) <= velocityTolerance.get();

    // 如果设定值为0，强制认为未就绪
    if (Math.abs(velocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }
    atGoal = atGoalDebouncer.calculate(inTolerance);
  }

  /** Stops the flywheel. */
  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    outputs.volts = 0.0; // 清零电压
    atGoal = false;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public double getGoalVelocity() {
    return fixspeed.getAsDouble();
  }

  public double getTOF() {
    return TOF.getAsDouble();
  }

  public void runVolts(double volts) {
    outputs.mode = FlywheelIOOutputMode.VOLTAGE;
    outputs.volts = volts; // 赋值给电压字段
    outputs.velocityRadsPerSec = 0.0; //
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(ShotCalculator.getInstance().getParameters().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand() {
    return runEnd(() -> runVelocity(fixspeed.getAsDouble()), this::stop);
  }

  public Command runFixedCommand(double velocity) {
    return runEnd(() -> runVelocity(velocity), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
