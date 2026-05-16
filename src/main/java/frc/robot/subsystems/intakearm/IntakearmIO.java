// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakearmIO {

  @AutoLog
  public static class IntakearmIOInputs {
    // TODO: add encoder
    boolean motorConnected = false;
    double positionRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double supplyCurrentAmps = 0.0;
    double torqueCurrentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  public static enum IntakearmIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP,
    HOMING
  }

  public static class IntakearmIOOutputs {

    public IntakearmIOOutputMode mode = IntakearmIOOutputMode.CLOSED_LOOP;
    // Closed loop control
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double volts = 0.0;
  }

  public default void updateInputs(IntakearmIOInputs inputs) {}

  public default void applyOutputs(IntakearmIOOutputs outputs) {}

  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void resetAngle(double Radius) {}
}
