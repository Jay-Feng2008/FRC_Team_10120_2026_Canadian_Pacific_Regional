package frc.robot.subsystems.climb;

public interface ClimbIO {
  class ClimbIOInputs {
    public boolean ClimbForwardMotorConnected = true;
    public boolean ClimbBackMotorConnected = true;

    public double ClimbPositionRads = 0.0;
    public double ClimbOmegaRadPerSec = 0.0;
    public double ClimbAlphaRadsPerSecSquared = 0.0;
    public double ClimbAppliedVolts = 0.0;
    public double ClimbSupplyCurrentAmps = 0.0;
    public double ClimbTorqueCurrentAmps = 0.0;
    public double ClimbTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ClimbIOInputs inputs) {}

  /** Run arm motor at volts */
  default void runVolts(double volts) {}

  /** Run to arm positionsetpoint with feedforward */
  default void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {}

  /** Run to arm positionsetpoint with motionmagic */
  // default void runMotionMagicClimbPosition(double angleRads, double omegaRadPerSec, double
  // alphaRadsPerSecSquared, double currentPositionRads) {}

  default void runMotionMagicArmPosition(double angleRads, double currentPositinRads) {}

  /** Configure arm PID */
  default void setClimbPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {}

  /** select arm or climb Mode */
  default void setPIDMode(boolean enable) {}

  /** resetPosition */
  default void resetPosition(double angleRads) {}

  /** Disable output to all motors */
  default void stop() {}

  default void switchBrake(boolean isBrake) {}

  default void runMotionMagicClimbPosition(double goal) {}

  //       default void runMotionMagicClimbPosition(double goal, double angleRads);
}
