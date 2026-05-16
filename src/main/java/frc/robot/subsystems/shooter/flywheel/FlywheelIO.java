// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;

    public boolean followerConnected;
    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;
    public double followerAppliedVoltage;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE
  }

  public static class FlywheelIOOutputs {
    // public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  /** Configure turret PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  default void runVelocity(double velocityRadsPerSec) {}
}
