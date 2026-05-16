// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wheelback;

import org.littletonrobotics.junction.AutoLog;

public interface WheelbackIO {
  @AutoLog
  public static class WheelbackIOInputs {
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

  public static enum WheelbackIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE
  }

  public static class WheelbackIOOutputs {
    // public WheelbackIOOutputMode mode = WheelbackIOOutputMode.COAST;
    public WheelbackIOOutputMode mode = WheelbackIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(WheelbackIOInputs inputs) {}

  default void applyOutputs(WheelbackIOOutputs outputs) {}

  /** Configure turret PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  default void runVelocity(double velocityRadsPerSec) {}
}
