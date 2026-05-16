package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoPilot2HubConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoAlignTestCommand2Hub extends Command {

  Drive m_Drive;
  PIDController m_HeadingController = new PIDController(3.0, 0, 0.1);
  // PIDController m_HeadingController = new PIDController(0.1, 0, 0.0);
  private boolean m_IsAtTarget = false;
  private boolean m_flip;
  private Pose2d m_ActiveTarget; // 用于执行的最终坐标

  public AutoAlignTestCommand2Hub(Drive drive, Pose2d _targetPose, boolean flip) {
    addRequirements(drive);
    m_Drive = drive;
    m_ActiveTarget = _targetPose;
    m_HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    m_flip = flip;
  }

  @Override
  public void initialize() {
    m_HeadingController.reset();
    m_IsAtTarget = false;

    if (m_flip) {
      m_ActiveTarget = AllianceFlipUtil.apply(m_ActiveTarget);
    } else {
      m_ActiveTarget = m_ActiveTarget;
    }
    // AutoPilotConstants
  }

  @Override
  public void execute() {
    moveToTarget(m_ActiveTarget);
  }

  void moveToTarget(Pose2d _targetPose) {

    ChassisSpeeds m_RobotChassisSpeeds = m_Drive.getChassisSpeeds();
    APResult output =
        AutoPilot2HubConstants.kAutopilot.calculate(
            m_Drive.getPose(), m_RobotChassisSpeeds, new APTarget(_targetPose)); // 要改
    m_IsAtTarget =
        AutoPilot2HubConstants.kAutopilot.atTarget(m_Drive.getPose(), new APTarget(_targetPose));
    LinearVelocity veloX = output.vx();
    LinearVelocity veloY = output.vy();
    double headingSpeed =
        m_HeadingController.calculate(
            m_Drive.getRotation().getRadians(),
            ShotCalculator.getInstance().getParameters().turretAngle().getRadians());
    // MathUtil.clamp(
    //     headingSpeed,
    //     -AutoPilot2HubConstants.kMaxAngularVelocity.in(Degrees),
    //     AutoPilot2HubConstants.kMaxAngularVelocity.in(Degrees));
    // Logger.recordOutput("AutoAlignTestCode/VeloX", veloX.in(MetersPerSecond));

    Logger.recordOutput("AutoAlignTestCode/IsAtTarget", m_IsAtTarget);
    if (!m_IsAtTarget)
      m_Drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              veloX.in(MetersPerSecond),
              veloY.in(MetersPerSecond),
              headingSpeed,
              m_Drive.getPose().getRotation()));
    else {
      m_Drive.stop();
    }
  }

  @Override
  public void end(boolean _interrupted) {
    m_Drive.stop();
  }

  @Override
  public boolean isFinished() {
    return m_IsAtTarget;
  }
}
