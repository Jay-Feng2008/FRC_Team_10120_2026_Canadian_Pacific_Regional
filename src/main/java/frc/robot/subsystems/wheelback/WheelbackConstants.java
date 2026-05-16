package frc.robot.subsystems.wheelback;

import edu.wpi.first.math.util.Units;

public class WheelbackConstants {

  public static final int wheelbackId = 6;

  public static final double wheelbackRadius = 0.18; // TODO
  public static final double wheelbackGearRatio = 1.26; // TODO

  public static final double kP = 14; // 100
  public static final double kI = 2.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 2.99; // 静态前馈
  public static final double kV = 0.052; //
  public static final double kA = 0; //

  public static final double velocityTolerance = 0.05;

  public static final double accelerationRadPerSecSq = Units.degreesToRadians(180); // 最大角加速度
  public static final double velocityRadPerSec = Units.degreesToRadians(180);

  // Sim
  // public static final double jkMetersSquared = 1.06328;

  // public static final double drumRadiusMeters = 0.494;
}
