package frc.robot.subsystems.intakearm;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class IntakearmConstants {

  public static final int IntakearmId = Ports.kIntakePivot;

  public static final double IntakearmGearRatio = 55.385; // TODO

  public static final double kP = 160; // 100
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 3.0; // 重力补偿
  public static final double kS = 0.000; // 静态前馈
  public static final double kV = 0; // 轨迹规划得到的目标速度
  public static final double kA = 0; // 轨迹规划得到的目标加速度

  public static final double accelerationRadPerSecSq = Units.degreesToRadians(360); // 最大角加速度
  public static final double velocityRadPerSec = Units.degreesToRadians(360);

  public static final double IntakearmMinAngle = Units.degreesToRadians(0); // TODO:
  public static final double IntakearmMaxAngle = Units.degreesToRadians(135);
  public static final double IntakearmInitialAngle = Units.degreesToRadians(0.0);

  public static final double Intakearmoffset =
      Units.degreesToRadians(0.0); // TODO: offset for Intakearm angle
  public static final double IntakearmtoleranceDeg = 0.05;
  public static final double homingVoltage = 1.0;

  public static final double IntakearmStallCurrent = 4.0; // TODO: Intakearm stall current

  // Sim
  // public static final double jkMetersSquared = 1.06328;

  // public static final double drumRadiusMeters = 0.494;
}
