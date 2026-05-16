package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputs inputs = new ClimbIOInputs();

  public double goal = 0.0;
  public boolean pidMode = true;
  public boolean neutralMode_brake = true;

  public Climb(ClimbIO io) {
    this.io = io;
    io.setClimbPID(
        ClimbConstants.kp,
        ClimbConstants.Ki,
        ClimbConstants.Kd,
        ClimbConstants.Ks,
        ClimbConstants.Kv,
        ClimbConstants.Ka,
        ClimbConstants.Kg,
        ClimbConstants.acceleration,
        ClimbConstants.velocity);
  }

  public void runGoal(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    io.runPositionSetpoint(angleRads, omegaRadPerSec, alphaRadsPerSecSquared, torque);
  }

  // public void runMotionMagicPosition(double angleRads, double omegaRadPerSec, double
  // alphaRadsPerSecSquared, double currentAngle) {
  //   goal = MathUtil.clamp(angleRads, ClimbConstants.ClimbMinimumAngle,
  // ClimbConstants.ClimbMaximumAngle);
  //   io.runMotionMagicClimbPosition(goal, omegaRadPerSec, alphaRadsPerSecSquared, currentAngle);//
  // / TorqueNumber.getAsDouble());
  // }

  public void runMotionMagicPosition(double angleRads) {
    goal =
        MathUtil.clamp(
            angleRads, ClimbConstants.ClimbMinimumAngle, ClimbConstants.ClimbMaximumAngle);
    io.runMotionMagicClimbPosition(goal);
  }

  public void extendhood() {
    goal = ClimberConstants.high_pose;
    io.runMotionMagicClimbPosition(goal);
  }

  public void climb() {
    io.runMotionMagicClimbPosition(0);
  }

  // public void runClimbPosition(double angleRads) {
  //   goal = MathUtil.clamp(angleRads, ClimbConstants.ClimbMinimumAngle,
  // ClimbConstants.ClimbMaximumAngle);
  //   io.runMotionMagicClimbPosition(goal);
  // }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void switchBrake() {
    neutralMode_brake = !neutralMode_brake;
    io.switchBrake(!neutralMode_brake);
  }

  public void stop() {
    io.stop();
  }

  public double getOmegaRadPerSec() {
    return inputs.ClimbOmegaRadPerSec;
  }

  public double getAlphaRadsPerSecSquared() {
    return inputs.ClimbAlphaRadsPerSecSquared;
  }

  public double getSupplyCurrentAmps() {
    return inputs.ClimbSupplyCurrentAmps;
  }

  public double getTorqueCurrentAmps() {
    return inputs.ClimbTorqueCurrentAmps;
  }

  public double getAngleRads() {
    return inputs.ClimbPositionRads;
  }

  public double getvolts() {
    return inputs.ClimbAppliedVolts;
  }

  public boolean atGoal() {
    return Math.abs(goal - getAngleRads()) < ClimbConstants.ClimbTolerance;
  }

  public void resetangle() {
    io.resetPosition(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    SmartDashboard.putNumber("ClimbOmegaRadPerSec()", getOmegaRadPerSec());
    SmartDashboard.putBoolean("atGoal", atGoal());
    SmartDashboard.putNumber("ClimbAngleRads", getAngleRads());
    SmartDashboard.putNumber("ClimbAppliedVolts", getAlphaRadsPerSecSquared());
    SmartDashboard.putNumber("ClimbSupplyCurrent", getSupplyCurrentAmps());
    SmartDashboard.putNumber("ClimbTorqueCurrent", getTorqueCurrentAmps());

    Logger.recordOutput("Climb/Velocity", inputs.ClimbOmegaRadPerSec);
    Logger.recordOutput("Climb/AtGoal", atGoal());
    Logger.recordOutput("Climb/Volts", getvolts());
    Logger.recordOutput("Climb/SupplyCurrent", getSupplyCurrentAmps());
    Logger.recordOutput("Climb/TorqueCurrent", getTorqueCurrentAmps());
  }
}
