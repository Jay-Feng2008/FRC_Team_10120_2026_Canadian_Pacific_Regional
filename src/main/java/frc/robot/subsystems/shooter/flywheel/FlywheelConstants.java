package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.util.Units;

public class FlywheelConstants {

  public static final int flywheelId = 15;

  public static final double flywheelRadius = 0.18; // TODO
  public static final double flywheelGearRatio = 1.26; // TODO

  public static final double kP = 16.3; // 100
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 3.55; // 静态前馈
  public static final double kV = 0.120; //
  public static final double kA = 0; //

  public static final double velocityTolerance = 10;

  public static final double accelerationRadPerSecSq = Units.degreesToRadians(180); // 最大角加速度
  public static final double velocityRadPerSec = Units.degreesToRadians(180);

  // Sim
  // public static final double jkMetersSquared = 1.06328;

  // public static final double drumRadiusMeters = 0.494;
}
