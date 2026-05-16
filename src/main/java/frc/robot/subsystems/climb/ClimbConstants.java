package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;

public class ClimbConstants {
  public static final int ClimbForwardMotorid = 16;
  public static final int ClimbBackMotorid = 18;

  // public static final double ClimbRadius = 0.09;             //待改
  public static final double ClimbGearRatio = 39.2;

  public static final double kp = 200; // 80;
  public static final double Ki = 10;
  public static final double Kd = 10.0; // 0.0;
  public static final double Kg = 4;
  public static final double Ks = 0;
  public static final double Kv = 0;
  public static final double Ka = 0;
  public static final double acceleration = 1.5;
  public static final double velocity = 0.5;

  public static final double ClimbMinimumAngle = Math.toRadians(-90); // radius
  public static final double ClimbMaximumAngle = Math.toRadians(90); // radius
  public static final double ClimbInitialAngle = 0.0;

  public static final double ClimbTestAngle = Math.toRadians(110);

  public static final double ClimbDownAngle = Math.toRadians(20);
  public static final double ClimbUpAngle = Math.toRadians(20);

  public static final double ClimbTolerance = Math.toRadians(7);

  public static final double testUpVolts = 5.0;
  public static final double testDownVolts = -5.0;

  // Sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0);
  public static final double jkMetersSquared = 1.06328;
  public static final double armLength = 0.5;
  public static final double loopPeriodSecs = 0.02;
}
