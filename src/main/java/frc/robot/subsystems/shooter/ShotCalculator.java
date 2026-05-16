// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
/// import  frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.GeomUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;

  public enum ShotTarget {
    Hub,
    up,
    down,
    CUSTOM
  }

  private ShotTarget currentTarget = ShotTarget.Hub;
  private Translation2d customTarget = new Translation2d();

  private Drive drive;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  // 增加一个初始化方法
  public void init(Drive drive) {
    this.drive = drive;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double wheelbackspeed,
      double flywheelSpeed,
      double lookaheadTurretToTargetDistance,
      boolean passing) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap distanceToBackwheelMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap backwheelToFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0.0;
    maxDistance = 15.60;
    phaseDelay = 0.03;

    distanceToBackwheelMap.put(181.0000, 170.8213);
    distanceToBackwheelMap.put(187.2000, 170.4476);
    distanceToBackwheelMap.put(193.4000, 170.5990);
    distanceToBackwheelMap.put(199.6000, 171.2926);
    distanceToBackwheelMap.put(205.8000, 172.5415);
    distanceToBackwheelMap.put(212.0000, 174.3555);
    distanceToBackwheelMap.put(218.2000, 176.7398);
    distanceToBackwheelMap.put(224.4000, 179.6958);
    distanceToBackwheelMap.put(230.6000, 183.2204);
    distanceToBackwheelMap.put(236.8000, 187.3064);
    distanceToBackwheelMap.put(243.0000, 191.9423);
    distanceToBackwheelMap.put(249.2000, 197.1122);
    distanceToBackwheelMap.put(255.4000, 202.7963);
    distanceToBackwheelMap.put(261.6000, 208.9708);
    distanceToBackwheelMap.put(267.8000, 215.6080);
    distanceToBackwheelMap.put(274.0000, 222.6769);
    distanceToBackwheelMap.put(280.2000, 230.1431);
    distanceToBackwheelMap.put(286.4000, 237.9692);
    distanceToBackwheelMap.put(292.6000, 246.1156);
    distanceToBackwheelMap.put(298.8000, 254.5399);
    distanceToBackwheelMap.put(305.0000, 263.1983);
    distanceToBackwheelMap.put(311.2000, 272.0454);
    distanceToBackwheelMap.put(317.4000, 281.0347);
    distanceToBackwheelMap.put(323.6000, 290.1193);
    distanceToBackwheelMap.put(329.8000, 299.2520);
    distanceToBackwheelMap.put(336.0000, 308.3857);
    distanceToBackwheelMap.put(342.2000, 317.4741);
    distanceToBackwheelMap.put(348.4000, 326.4719);
    distanceToBackwheelMap.put(354.6000, 335.3349);
    distanceToBackwheelMap.put(360.8000, 344.0208);
    distanceToBackwheelMap.put(367.0000, 352.4895);
    distanceToBackwheelMap.put(373.2000, 360.7029);
    distanceToBackwheelMap.put(379.4000, 368.6254);
    distanceToBackwheelMap.put(385.6000, 376.2245);
    distanceToBackwheelMap.put(391.8000, 383.4704);
    distanceToBackwheelMap.put(398.0000, 390.3364);
    distanceToBackwheelMap.put(404.2000, 396.7990);
    distanceToBackwheelMap.put(410.4000, 402.8380);
    distanceToBackwheelMap.put(416.6000, 408.4364);
    distanceToBackwheelMap.put(422.8000, 413.5806);
    distanceToBackwheelMap.put(429.0000, 418.2602);
    distanceToBackwheelMap.put(435.2000, 422.4681);
    distanceToBackwheelMap.put(441.4000, 426.2002);
    distanceToBackwheelMap.put(447.6000, 429.4555);

    // interpolate flywheel speed from interpolated backwheel speed.
    backwheelToFlywheelSpeedMap.put(177.0000, 163.2670);
    backwheelToFlywheelSpeedMap.put(181.6140, 162.3042);
    backwheelToFlywheelSpeedMap.put(186.2281, 161.3125);
    backwheelToFlywheelSpeedMap.put(190.8421, 160.2930);
    backwheelToFlywheelSpeedMap.put(195.4561, 159.2470);
    backwheelToFlywheelSpeedMap.put(200.0702, 158.1756);
    backwheelToFlywheelSpeedMap.put(204.6842, 157.0802);
    backwheelToFlywheelSpeedMap.put(209.2982, 155.9621);
    backwheelToFlywheelSpeedMap.put(213.9123, 154.8227);
    backwheelToFlywheelSpeedMap.put(218.5263, 153.6634);
    backwheelToFlywheelSpeedMap.put(223.1404, 152.4857);
    backwheelToFlywheelSpeedMap.put(227.7544, 151.2911);
    backwheelToFlywheelSpeedMap.put(232.3684, 150.0811);
    backwheelToFlywheelSpeedMap.put(236.9825, 148.8574);
    backwheelToFlywheelSpeedMap.put(241.5965, 147.6215);
    backwheelToFlywheelSpeedMap.put(246.2105, 146.3750);
    backwheelToFlywheelSpeedMap.put(250.8246, 145.1197);
    backwheelToFlywheelSpeedMap.put(255.4386, 143.8571);
    backwheelToFlywheelSpeedMap.put(260.0526, 142.5889);
    backwheelToFlywheelSpeedMap.put(264.6667, 141.3169);
    backwheelToFlywheelSpeedMap.put(269.2807, 140.0427);
    backwheelToFlywheelSpeedMap.put(273.8947, 138.7679);
    backwheelToFlywheelSpeedMap.put(278.5088, 137.4944);
    backwheelToFlywheelSpeedMap.put(283.1228, 136.2238);
    backwheelToFlywheelSpeedMap.put(287.7368, 134.9577);
    backwheelToFlywheelSpeedMap.put(292.3509, 133.6979);
    backwheelToFlywheelSpeedMap.put(296.9649, 132.4460);
    backwheelToFlywheelSpeedMap.put(301.5789, 131.2035);
    backwheelToFlywheelSpeedMap.put(306.1930, 129.9722);
    backwheelToFlywheelSpeedMap.put(310.8070, 128.7535);
    backwheelToFlywheelSpeedMap.put(315.4211, 127.5491);
    backwheelToFlywheelSpeedMap.put(320.0351, 126.3604);
    backwheelToFlywheelSpeedMap.put(324.6491, 125.1889);
    backwheelToFlywheelSpeedMap.put(329.2632, 124.0361);
    backwheelToFlywheelSpeedMap.put(333.8772, 122.9034);
    backwheelToFlywheelSpeedMap.put(338.4912, 121.7920);
    backwheelToFlywheelSpeedMap.put(343.1053, 120.7034);
    backwheelToFlywheelSpeedMap.put(347.7193, 119.6387);
    backwheelToFlywheelSpeedMap.put(352.3333, 118.5991);
    backwheelToFlywheelSpeedMap.put(356.9474, 117.5858);
    backwheelToFlywheelSpeedMap.put(361.5614, 116.5999);
    backwheelToFlywheelSpeedMap.put(366.1754, 115.6424);
    backwheelToFlywheelSpeedMap.put(370.7895, 114.7142);
    backwheelToFlywheelSpeedMap.put(375.4035, 113.8163);
    backwheelToFlywheelSpeedMap.put(380.0175, 112.9495);
    backwheelToFlywheelSpeedMap.put(384.6316, 112.1145);
    backwheelToFlywheelSpeedMap.put(389.2456, 111.3121);
    backwheelToFlywheelSpeedMap.put(393.8596, 110.5430);
    backwheelToFlywheelSpeedMap.put(398.4737, 109.8076);
    backwheelToFlywheelSpeedMap.put(403.0877, 109.1065);
    backwheelToFlywheelSpeedMap.put(407.7018, 108.4402);
    backwheelToFlywheelSpeedMap.put(412.3158, 107.8089);
    backwheelToFlywheelSpeedMap.put(416.9298, 107.2131);
    backwheelToFlywheelSpeedMap.put(421.5439, 106.6528);
    backwheelToFlywheelSpeedMap.put(426.1579, 106.1284);
    backwheelToFlywheelSpeedMap.put(430.7719, 105.6400);
    backwheelToFlywheelSpeedMap.put(435.3860, 105.1874);
    backwheelToFlywheelSpeedMap.put(440.0000, 104.7708);
    // timeOfFlightMap.put(5.68, 1.6);
    // timeOfFlightMap.put(4.55, 1.5);
    timeOfFlightMap.put(1.6940, 1.2485);
    timeOfFlightMap.put(1.7245, 1.2397);
    timeOfFlightMap.put(1.7550, 1.2305);
    timeOfFlightMap.put(1.7854, 1.2210);
    timeOfFlightMap.put(1.8159, 1.2115);
    timeOfFlightMap.put(1.8464, 1.2021);
    timeOfFlightMap.put(1.8769, 1.1929);
    timeOfFlightMap.put(1.9074, 1.1840);
    timeOfFlightMap.put(1.9378, 1.1756);
    timeOfFlightMap.put(1.9683, 1.1677);
    timeOfFlightMap.put(1.9988, 1.1607);
    timeOfFlightMap.put(2.0293, 1.1545);
    timeOfFlightMap.put(2.0598, 1.1493);
    timeOfFlightMap.put(2.0902, 1.1453);
    timeOfFlightMap.put(2.1207, 1.1426);
    timeOfFlightMap.put(2.1512, 1.1413);
    timeOfFlightMap.put(2.1817, 1.1416);
    timeOfFlightMap.put(2.2122, 1.1432);
    timeOfFlightMap.put(2.2426, 1.1462);
    timeOfFlightMap.put(2.2731, 1.1504);
    timeOfFlightMap.put(2.3036, 1.1555);
    timeOfFlightMap.put(2.3341, 1.1616);
    timeOfFlightMap.put(2.3646, 1.1684);
    timeOfFlightMap.put(2.3950, 1.1757);
    timeOfFlightMap.put(2.4255, 1.1836);
    timeOfFlightMap.put(2.4560, 1.1918);
    timeOfFlightMap.put(2.4865, 1.2002);
    timeOfFlightMap.put(2.5170, 1.2086);
    timeOfFlightMap.put(2.5474, 1.2169);
    timeOfFlightMap.put(2.5779, 1.2250);
    timeOfFlightMap.put(2.6084, 1.2328);
    timeOfFlightMap.put(2.6389, 1.2401);
    timeOfFlightMap.put(2.6694, 1.2470);
    timeOfFlightMap.put(2.6998, 1.2534);
    timeOfFlightMap.put(2.7303, 1.2595);
    timeOfFlightMap.put(2.7608, 1.2651);
    timeOfFlightMap.put(2.7913, 1.2703);
    timeOfFlightMap.put(2.8218, 1.2752);
    timeOfFlightMap.put(2.8522, 1.2796);
    timeOfFlightMap.put(2.8827, 1.2837);
    timeOfFlightMap.put(2.9132, 1.2875);
    timeOfFlightMap.put(2.9437, 1.2909);
    timeOfFlightMap.put(2.9742, 1.2939);
    timeOfFlightMap.put(3.0046, 1.2966);
    timeOfFlightMap.put(3.0351, 1.2990);
    timeOfFlightMap.put(3.0656, 1.3011);
    timeOfFlightMap.put(3.0961, 1.3029);
    timeOfFlightMap.put(3.1266, 1.3044);
    timeOfFlightMap.put(3.1570, 1.3056);
    timeOfFlightMap.put(3.1875, 1.3065);
    timeOfFlightMap.put(3.2180, 1.3071);
  }

  public ShootingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(drive.getPose().getX()) > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    // Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    Pose2d estimatedPose = drive.getPose();
    ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();
    // Advance pose by phase delay
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Translation2d target = getCurrentTarget();
    Pose2d turretPosition = estimatedPose;
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drive.getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX = robotVelocity.vxMetersPerSecond;
    double turretVelocityY = robotVelocity.vyMetersPerSecond;

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    double backwheelVelocity = distanceToBackwheelMap.get(lookaheadTurretToTargetDistance * 100);
    double flywheelVelocity = backwheelToFlywheelSpeedMap.get(backwheelVelocity);
    latestParameters =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            backwheelVelocity,
            flywheelVelocity,
            lookaheadTurretToTargetDistance,
            false); // TODO: passing calculation

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput("ShotCalculator/BackwheelVelocity", backwheelVelocity);
    Logger.recordOutput("ShotCalculator/FlywheelVelocity", flywheelVelocity);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }

  public void setTarget(ShotTarget target) {
    if (this.currentTarget != target) {
      this.currentTarget = target;
      clearShootingParameters(); // 非常重要：清缓存
    }
  }

  public void setCustomTarget(Translation2d target) {
    this.customTarget = target;
    this.currentTarget = ShotTarget.CUSTOM;
    clearShootingParameters();
  }

  private Translation2d getCurrentTarget() {
    return switch (currentTarget) {
      case Hub -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      case down -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      case up -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      case CUSTOM -> customTarget;
    };
  }
}
