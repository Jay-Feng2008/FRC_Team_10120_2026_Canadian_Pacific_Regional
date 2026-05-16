// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;
  public static boolean disableHAL = false; //
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double ControllerDeadband = 0.04;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // ---------------- Autopilot (vendor lib) ----------------
  private static final APConstraints kAutopilotConstraints =
      new APConstraints().withAcceleration(10.0).withJerk(100);

  private static final APProfile kAutopilotProfile =
      new APProfile(kAutopilotConstraints)
          .withErrorXY(Units.Centimeters.of(2))
          .withErrorTheta(Units.Degrees.of(2))
          .withBeelineRadius(Units.Centimeters.of(10));

  /** Shared Autopilot instance (create once, reuse everywhere). */
  public static final Autopilot kAutopilot = new Autopilot(kAutopilotProfile);

  public static final class XFieldConstants {
    public static final double HUB_HEIGHT_METERS = 2.64;
    public static final Translation2d RED_HUB_LOCATION = new Translation2d(11.917, 4.030);
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.623, 4.030);

    /** Estimated field center from hub locations (used for symmetry helpers). */
    public static final Translation2d FIELD_CENTER =
        new Translation2d(
            (RED_HUB_LOCATION.getX() + BLUE_HUB_LOCATION.getX()) / 2.0,
            (RED_HUB_LOCATION.getY() + BLUE_HUB_LOCATION.getY()) / 2.0);

    // ---------------- Trench alignment helpers ----------------
    /**
     * Standoff distance before the trench along the chosen approach line.
     *
     * <p>This is the "坡前指定位置" distance.
     */
    public static final double TRENCH_PRE_DISTANCE_METERS = 1.20;

    /** Trench usable alignment length along the trench axis (meters). */
    public static final double TRENCH_LENGTH_METERS = 0.10;

    /**
     * Base trench center (blue alliance, lower-left one) in field coordinates.
     *
     * <p>TODO: Set this to your measured field value.
     */
    public static final Translation2d BLUE_LL_TRENCH_CENTER = new Translation2d(4.623, 0.640);

    /**
     * Base trench axis heading (blue alliance, lower-left trench). This is the direction along the
     * trench length (the finite segment direction).
     *
     * <p>If your trench is "horizontal", set this to 0 deg. If it's "vertical", set to 90 deg.
     *
     * <p>IMPORTANT: Trench centers are symmetric about {@link #FIELD_CENTER} midlines, but trench
     * axis orientation is NOT assumed to change with the centers. The field trenches are parallel
     * in global field coordinates (e.g., both blue-side bumps share the same axis).
     */
    public static final Rotation2d BLUE_LL_TRENCH_AXIS_HEADING = Rotation2d.fromDegrees(90.0);

    private static Translation2d mirrorAboutXMidline(
        Translation2d point, Translation2d fieldCenter) {
      // Mirror across the vertical midline x = fieldCenter.x
      return new Translation2d(2.0 * fieldCenter.getX() - point.getX(), point.getY());
    }

    private static Translation2d mirrorAboutYMidline(
        Translation2d point, Translation2d fieldCenter) {
      // Mirror across the horizontal midline y = fieldCenter.y
      return new Translation2d(point.getX(), 2.0 * fieldCenter.getY() - point.getY());
    }

    /** Snap a field heading to the nearest multiple of 90 degrees. */
    private static Rotation2d snapTo90Deg(Rotation2d heading) {
      double deg = heading.getDegrees();
      double snappedDeg = Math.round(deg / 90.0) * 90.0;
      return Rotation2d.fromDegrees(snappedDeg);
    }

    /** Represents an approach reference segment for a trench (finite length). */
    public record TrenchApproachLine(
        Translation2d trenchCenter, Rotation2d approachHeading, double halfLengthMeters) {
      public Translation2d dirUnit() {
        return new Translation2d(approachHeading.getCos(), approachHeading.getSin());
      }

      /** Unit vector along the trench axis (perpendicular to approach direction). */
      public Translation2d axisUnit() {
        Rotation2d axisHeading = approachHeading.plus(Rotation2d.fromRotations(0.25)); // +90deg
        return new Translation2d(axisHeading.getCos(), axisHeading.getSin());
      }

      /** Closest point on the finite segment to the given point. */
      public Translation2d closestPointOnSegment(Translation2d point) {
        Translation2d a = axisUnit();
        Translation2d d = point.minus(trenchCenter);
        double t = d.getX() * a.getX() + d.getY() * a.getY(); // projection onto axis
        t = Math.max(-halfLengthMeters, Math.min(halfLengthMeters, t));
        return trenchCenter.plus(a.times(t));
      }

      /** Signed distance along the line direction from trenchCenter to point. */
      public double along(Translation2d point) {
        Translation2d d = point.minus(trenchCenter);
        Translation2d u = dirUnit();
        return d.getX() * u.getX() + d.getY() * u.getY();
      }

      /** Distance from the point to this finite segment (meters). */
      public double distanceToSegment(Translation2d point) {
        return point.minus(closestPointOnSegment(point)).getNorm();
      }

      /** Returns the "pre-trench" pose for this line. */
      public Pose2d preTrenchPose(double preDistanceMeters) {
        return preTrenchPose(preDistanceMeters, trenchCenter);
      }

      /** Returns the "pre-trench" pose using a chosen point on the segment. */
      public Pose2d preTrenchPose(double preDistanceMeters, Translation2d pointOnSegment) {
        Translation2d u = dirUnit();
        Translation2d prePoint = pointOnSegment.minus(u.times(preDistanceMeters));
        return new Pose2d(prePoint, approachHeading);
      }
    }

    /**
     * 4 trench centers, generated by mirror symmetry about the field midlines (x = {@link
     * #FIELD_CENTER}.x and y = {@link #FIELD_CENTER}.y).
     */
    public static final Translation2d[] TRENCH_CENTERS =
        new Translation2d[] {
          BLUE_LL_TRENCH_CENTER,
          mirrorAboutXMidline(BLUE_LL_TRENCH_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(BLUE_LL_TRENCH_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(
              mirrorAboutXMidline(BLUE_LL_TRENCH_CENTER, FIELD_CENTER), FIELD_CENTER)
        };

    /** 4 trench axis headings (direction along trench length), NOT changed with the centers. */
    public static final Rotation2d[] TRENCH_AXIS_HEADINGS =
        new Rotation2d[] {
          BLUE_LL_TRENCH_AXIS_HEADING,
          BLUE_LL_TRENCH_AXIS_HEADING,
          BLUE_LL_TRENCH_AXIS_HEADING,
          BLUE_LL_TRENCH_AXIS_HEADING
        };

    /**
     * 8 approach reference lines: for each trench, we provide two approach directions normal to the
     * trench axis (axis +90 and axis -90). This matches the "8 条预瞄参考线" idea.
     *
     * <p>IMPORTANT: Segment direction = trench axis; Pose heading = approach direction.
     */
    public static final TrenchApproachLine[] TRENCH_APPROACH_LINES =
        new TrenchApproachLine[] {
          // trench 0
          new TrenchApproachLine(
              TRENCH_CENTERS[0],
              TRENCH_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[0],
              TRENCH_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          // trench 1
          new TrenchApproachLine(
              TRENCH_CENTERS[1],
              TRENCH_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[1],
              TRENCH_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          // trench 2
          new TrenchApproachLine(
              TRENCH_CENTERS[2],
              TRENCH_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[2],
              TRENCH_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          // trench 3
          new TrenchApproachLine(
              TRENCH_CENTERS[3],
              TRENCH_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[3],
              TRENCH_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0)
        };

    /**
     * Returns the best "pre-trench" alignment pose by snapping to the nearest approach reference
     * line.
     *
     * <p>Selection rule:
     *
     * <ul>
     *   <li>Pick the line with smallest perpendicular distance to the robot translation
     *   <li>Tie-breaker prefers the line where the robot is \"behind\" the trench center relative
     *       to the line direction (so the pre-pose is reachable without flipping 180°)
     * </ul>
     */
    public static Pose2d getNearestTrenchPrePose(Pose2d robotPose) {
      Translation2d p = robotPose.getTranslation();
      TrenchApproachLine best = null;
      double bestDist = Double.POSITIVE_INFINITY;
      boolean bestBehind = false;
      Translation2d bestClosest = null;

      for (TrenchApproachLine line : TRENCH_APPROACH_LINES) {
        Translation2d closest = line.closestPointOnSegment(p);
        double dist = line.distanceToSegment(p);
        // behind = robot is "before" the segment along the approach direction
        boolean behind =
            p.minus(closest).getX() * line.dirUnit().getX()
                    + p.minus(closest).getY() * line.dirUnit().getY()
                < 0.0;
        if (dist < bestDist - 1e-9) {
          best = line;
          bestDist = dist;
          bestBehind = behind;
          bestClosest = closest;
        } else if (Math.abs(dist - bestDist) <= 1e-9) {
          // Tie-breaker: prefer behind=true
          if (behind && !bestBehind) {
            best = line;
            bestBehind = true;
            bestClosest = closest;
          }
        }
      }

      if (best == null) {
        return robotPose;
      }

      // Keep translation on the chosen line, but snap the traverse heading to a 90° multiple.
      Translation2d u = best.dirUnit();
      Translation2d prePoint = bestClosest.minus(u.times(TRENCH_PRE_DISTANCE_METERS));
      return new Pose2d(prePoint, snapTo90Deg(best.approachHeading()));
    }

    /** ---------------- Bump alignment helpers ---------------- */
    /** Standoff distance before the bump along the chosen approach line (meters). */
    public static final double BUMP_PRE_DISTANCE_METERS = 1.0;

    /** Bump usable alignment length along the bump axis (meters). */
    public static final double BUMP_LENGTH_METERS = 0.5;

    /**
     * Base bump center (blue alliance, lower-left one) in field coordinates.
     *
     * <p>TODO: Set this to your measured field value.
     */
    public static final Translation2d BLUE_LL_BUMP_CENTER = new Translation2d(4.633, 2.480);

    /**
     * Base bump axis heading (blue alliance, lower-left bump). This is the direction along the bump
     * length (the finite segment direction).
     *
     * <p>IMPORTANT: Bump centers are symmetric about {@link #FIELD_CENTER} midlines, but bump axis
     * orientation is NOT assumed to change with the centers.
     */
    public static final Rotation2d BLUE_LL_BUMP_AXIS_HEADING = Rotation2d.fromDegrees(90.0);

    /** Represents an approach reference segment for a bump (finite length). */
    public record BumpApproachLine(
        Translation2d bumpCenter, Rotation2d approachHeading, double halfLengthMeters) {
      public Translation2d dirUnit() {
        return new Translation2d(approachHeading.getCos(), approachHeading.getSin());
      }

      /** Unit vector along the bump axis (perpendicular to approach direction). */
      public Translation2d axisUnit() {
        Rotation2d axisHeading = approachHeading.plus(Rotation2d.fromRotations(0.25)); // +90deg
        return new Translation2d(axisHeading.getCos(), axisHeading.getSin());
      }

      /** Closest point on the finite segment to the given point. */
      public Translation2d closestPointOnSegment(Translation2d point) {
        Translation2d a = axisUnit();
        Translation2d d = point.minus(bumpCenter);
        double t = d.getX() * a.getX() + d.getY() * a.getY(); // projection onto axis
        t = Math.max(-halfLengthMeters, Math.min(halfLengthMeters, t));
        return bumpCenter.plus(a.times(t));
      }

      /** Distance from the point to this finite segment (meters). */
      public double distanceToSegment(Translation2d point) {
        return point.minus(closestPointOnSegment(point)).getNorm();
      }
    }

    /** 4 bump centers, generated by mirror symmetry about the field midlines. */
    public static final Translation2d[] BUMP_CENTERS =
        new Translation2d[] {
          BLUE_LL_BUMP_CENTER,
          mirrorAboutXMidline(BLUE_LL_BUMP_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(BLUE_LL_BUMP_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(mirrorAboutXMidline(BLUE_LL_BUMP_CENTER, FIELD_CENTER), FIELD_CENTER)
        };

    /** 4 bump axis headings (direction along bump length), NOT changed with the centers. */
    public static final Rotation2d[] BUMP_AXIS_HEADINGS =
        new Rotation2d[] {
          BLUE_LL_BUMP_AXIS_HEADING,
          BLUE_LL_BUMP_AXIS_HEADING,
          BLUE_LL_BUMP_AXIS_HEADING,
          BLUE_LL_BUMP_AXIS_HEADING
        };

    /** 8 approach reference lines for bumps. */
    public static final BumpApproachLine[] BUMP_APPROACH_LINES =
        new BumpApproachLine[] {
          new BumpApproachLine(
              BUMP_CENTERS[0],
              BUMP_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[0],
              BUMP_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[1],
              BUMP_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[1],
              BUMP_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[2],
              BUMP_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[2],
              BUMP_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[3],
              BUMP_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[3],
              BUMP_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0)
        };

    /**
     * Returns the best "pre-bump" alignment pose by snapping to the nearest bump approach segment.
     *
     * <p>NO 90° heading snap. Returned pose rotation equals chosen approach heading.
     */
    public static Pose2d getNearestBumpPrePose(Pose2d robotPose) {
      Translation2d p = robotPose.getTranslation();
      BumpApproachLine best = null;
      double bestDist = Double.POSITIVE_INFINITY;
      boolean bestBehind = false;
      Translation2d bestClosest = null;

      for (BumpApproachLine line : BUMP_APPROACH_LINES) {
        Translation2d closest = line.closestPointOnSegment(p);
        double dist = line.distanceToSegment(p);
        boolean behind =
            p.minus(closest).getX() * line.dirUnit().getX()
                    + p.minus(closest).getY() * line.dirUnit().getY()
                < 0.0;
        if (dist < bestDist - 1e-9) {
          best = line;
          bestDist = dist;
          bestBehind = behind;
          bestClosest = closest;
        } else if (Math.abs(dist - bestDist) <= 1e-9) {
          if (behind && !bestBehind) {
            best = line;
            bestBehind = true;
            bestClosest = closest;
          }
        }
      }

      if (best == null) {
        return robotPose;
      }

      Translation2d u = best.dirUnit();
      Translation2d prePoint = bestClosest.minus(u.times(BUMP_PRE_DISTANCE_METERS));
      // return new Pose2d(prePoint, best.approachHeading().plus(Rotation2d.k180deg));
      return new Pose2d(prePoint, robotPose.getRotation());
      // return new Pose2d(prePoint, best.approachHeading());
    }

    public static final Translation2d getHubLocation(Alliance alliance) {

      return alliance == Alliance.Red ? RED_HUB_LOCATION : BLUE_HUB_LOCATION;
    }

    // ---------------- Lob / pass targets ----------------
    /**
     * Lob/pass target points for BLUE alliance (field coordinates).
     *
     * <p>TODO: Replace these placeholders with measured coordinates for your strategy.
     */
    public static final Translation2d[] BLUE_LOB_TARGETS =
        new Translation2d[] {
          new Translation2d(4.5, 5.58), new Translation2d(4.5, 2.31),
        };

    /** Lob/pass target points for RED alliance, mirrored from BLUE across the X midline. */
    public static final Translation2d[] RED_LOB_TARGETS =
        new Translation2d[] {
          mirrorAboutXMidline(BLUE_LOB_TARGETS[0], FIELD_CENTER),
          mirrorAboutXMidline(BLUE_LOB_TARGETS[1], FIELD_CENTER),
        };

    /** Returns the nearest lob/pass target point for the given alliance. */
    public static Translation2d getNearestLobTarget(
        Translation2d robotTranslation, Alliance alliance) {
      Translation2d[] candidates = alliance == Alliance.Red ? RED_LOB_TARGETS : BLUE_LOB_TARGETS;
      Translation2d best = candidates[0];
      double bestDist = robotTranslation.getDistance(best);
      for (int i = 1; i < candidates.length; i++) {
        double d = robotTranslation.getDistance(candidates[i]);
        if (d < bestDist) {
          best = candidates[i];
          bestDist = d;
        }
      }
      return best;
    }
  }

  public final class AutoPilotConstants {

    private static final APConstraints kConstraints =
        new APConstraints().withAcceleration(8.0).withJerk(4.0);
    public static final Angle kMaxAngularVelocity = Degrees.of(360.0);
    private static final APProfile kProfile =
        new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(3))
            .withErrorTheta(Degrees.of(1))
            .withBeelineRadius(Centimeters.of(1));
    public static final Autopilot kAutopilot = new Autopilot(kProfile);
  }

  public final class AutoPilot2HubConstants {

    private static final APConstraints kConstraints =
        new APConstraints().withAcceleration(8.0).withJerk(4.0).withVelocity(1.0);
    public static final Angle kMaxAngularVelocity = Degrees.of(360.0);
    private static final APProfile kProfile =
        new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(3))
            .withErrorTheta(Degrees.of(1))
            .withBeelineRadius(Centimeters.of(1));
    public static final Autopilot kAutopilot = new Autopilot(kProfile);
  }

  public static class ClimberConstants {
    public static final int FRONT_ID = 13;
    public static final int BACK_ID = 1;

    public static final double high_pose = 1.1;

    public static final Pose2d CLIMB_RIGHT_FRONT_POSE =
        new Pose2d(0.921, 2.761, Rotation2d.fromDegrees(0)); // TODO 现场修改 右侧车头向前
    public static final Pose2d CLIMB_RIGHT_BACK_POSE =
        new Pose2d(0.921, 2.761, Rotation2d.fromDegrees(180)); // TODO 现场修改 右侧车头向后
    public static final Pose2d CLIMB_LEFT_POSE =
        new Pose2d(1.396, 3.713, Rotation2d.fromDegrees(180)); // TODO 现场修改
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs().withStatorCurrentLimit(80);

    public static final MotorOutputConfigs FRONT_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs BACK_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final Voltage CLIMB_VOLTAGE = Volts.of(-12);
    public static final Voltage STOW_VOLTAGE = Volts.of(-3);
    public static final Voltage EXTEND_VOLTAGE = Volts.of(3);
    public static final Voltage ZERO_VOLTAGE = Volts.of(-1);

    public static final Current STALL_CURRENT = Amps.of(20);
    public static final AngularVelocity STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(0.2);

    public static final Angle CLIMB_POSITION = Rotations.of(25);
    public static final Angle AUTO_CLIMB_POSITION = Rotations.of(35);
    public static final Angle STOW_POSITION = Rotations.of(0.1);
    public static final Angle EXTEND_POSITION_FRONT = Rotations.of(43);
    public static final Angle EXTEND_POSITION_BACK = Rotations.of(43);

    // volts / rotation diff
    public static final double DIFF_KP = 0.0;

    private static final ControlConstants CLIMB_ALIGN_BASE_CONSTANTS_TRANSLATION =
        new ControlConstants().withPID(2, 0, 0).withTolerance(0.02);

    public static final TunableControlConstants CLIMB_ALIGN_CONSTANTS_TRANSLATION =
        new TunableControlConstants(
            "Climber/AlignTranslation", CLIMB_ALIGN_BASE_CONSTANTS_TRANSLATION);

    private static final ControlConstants CLIMB_ALIGN_BASE_CONSTANTS_ROTATION =
        new ControlConstants()
            .withPID(2, 0, 0)
            .withTolerance(Degrees.of(3).in(Radians))
            .withContinuous(-Math.PI, Math.PI);

    public static final TunableControlConstants CLIMB_ALIGN_CONSTANTS_ROTATION =
        new TunableControlConstants("Climber/AlignRotation", CLIMB_ALIGN_BASE_CONSTANTS_ROTATION);
  }
  /** Tuning for automatic bump traversal (SmashBumpCommand). */
  public static final class BumpCommandConstants {
    // ALIGN completion tolerances
    public static final double ALIGN_XY_TOL_METERS = 0.05;
    public static final double ALIGN_THETA_TOL_DEG = 5.0;

    // Autopilot target config during ALIGN
    public static final double ALIGN_END_VELOCITY_MPS = 2.;

    // Sprint behavior (field-relative along approach direction)
    public static final double SPRINT_SPEED_MPS = 2.;

    // Heading hold during sprint
    public static final double HEADING_KP = 5.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.2;

    // public static final double HEADING_KP = 0.1;
    // public static final double HEADING_KI = 0.0;
    // public static final double HEADING_KD = 0.0;

    /** After tilt clears (downhill), keep sprinting for this long then stop. */
    public static final double DOWNHILL_TIME_SEC = 0.1;

    private BumpCommandConstants() {}
  }
}
