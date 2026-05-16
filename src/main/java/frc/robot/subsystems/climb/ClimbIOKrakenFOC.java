package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public class ClimbIOKrakenFOC implements ClimbIO {
  // hardwares
  private final TalonFX ClimbForwardMotor;

  // Status Signals
  private final StatusSignal<Angle> ClimbPosition_F;
  private final StatusSignal<AngularVelocity> ClimbVelocity_F;
  private final StatusSignal<Voltage> ClimbAppliedVolts_F;
  private final StatusSignal<Current> ClimbSupplyCurrent_F;
  private final StatusSignal<Current> ClimbTorqueCurrent_F;
  private final StatusSignal<AngularAcceleration> Climbacceleration_F;
  private final StatusSignal<Temperature> ClimbTempCelsius_F;

  // private final StatusSignal<Angle> ClimbPosition_B;
  // private final StatusSignal<AngularVelocity> ClimbVelocity_B;
  // private final StatusSignal<Voltage> ClimbAppliedVolts_B;
  // private final StatusSignal<Current> ClimbSupplyCurrent_B;
  // private final StatusSignal<Current> ClimbTorqueCurrent_B;
  // private final StatusSignal<AngularAcceleration> Climbacceleration_B;
  // private final StatusSignal<Temperature> ClimbTempCelsius_B;

  // private final Follower follower = new Follower(ClimbConstants.ClimbForwardMotorid, false);

  // private double ClimbRadius = ClimbConstants.ClimbRadius;

  private final TalonFXConfiguration ClimbFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final MotionMagicVoltage motionmagicControl = new MotionMagicVoltage(0.0);
  private final NeutralOut neutralControl = new NeutralOut();
  // private final CoastOut neutralControl = new CoastOut();

  public ClimbIOKrakenFOC() {
    ClimbForwardMotor = new TalonFX(ClimbConstants.ClimbForwardMotorid, "rio");
    // ClimbBackMotor = new TalonFX(ClimbConstants.ClimbBackMotorid, "canivore10120");

    ClimbFXConfiguration.Feedback.SensorToMechanismRatio = ClimbConstants.ClimbGearRatio;

    ClimbFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ClimbFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ClimbFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /*Torque Current Limiting */
    // armFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;
    // armFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0;
    // armFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    ClimbFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    ClimbFXConfiguration.CurrentLimits.StatorCurrentLimit = 80.0;

    /* Current Limiting */
    ClimbFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    ClimbFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    ClimbFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
    ClimbFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    /* Open and Closed Loop Ramping */
    ClimbFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    ClimbFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

    ClimbForwardMotor.getConfigurator().apply(ClimbFXConfiguration);
    // ClimbBackMotor.getConfigurator().apply(ClimbFXConfiguration);

    resetPosition(ClimbConstants.ClimbInitialAngle);

    // Set signals
    ClimbPosition_F = ClimbForwardMotor.getPosition();
    ClimbVelocity_F = ClimbForwardMotor.getVelocity();
    ClimbAppliedVolts_F = ClimbForwardMotor.getMotorVoltage();
    ClimbSupplyCurrent_F = ClimbForwardMotor.getSupplyCurrent();
    ClimbTorqueCurrent_F = ClimbForwardMotor.getTorqueCurrent();
    Climbacceleration_F = ClimbForwardMotor.getAcceleration();
    ClimbTempCelsius_F = ClimbForwardMotor.getDeviceTemp();

    //  ClimbPosition_B =ClimbBackMotor.getPosition();
    //  ClimbVelocity_B =ClimbBackMotor.getVelocity();
    //  ClimbAppliedVolts_B =ClimbBackMotor.getMotorVoltage();
    //  ClimbSupplyCurrent_B =ClimbBackMotor.getSupplyCurrent();
    //  ClimbTorqueCurrent_B =ClimbBackMotor.getTorqueCurrent();
    //  Climbacceleration_B =ClimbBackMotor.getAcceleration();
    //  ClimbTempCelsius_B =ClimbBackMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        ClimbPosition_F,
        ClimbVelocity_F,
        ClimbAppliedVolts_F,
        ClimbSupplyCurrent_F,
        ClimbTorqueCurrent_F,
        Climbacceleration_F,
        ClimbTempCelsius_F

        //  ClimbPosition_B,
        //  ClimbVelocity_B,
        //  ClimbAppliedVolts_B,
        //  ClimbSupplyCurrent_B,
        //  ClimbTorqueCurrent_B,
        //  Climbacceleration_B,
        //  ClimbTempCelsius_B

        );
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.ClimbForwardMotorConnected =
        BaseStatusSignal.refreshAll(
                ClimbPosition_F,
                ClimbVelocity_F,
                ClimbAppliedVolts_F,
                ClimbSupplyCurrent_F,
                ClimbTorqueCurrent_F,
                Climbacceleration_F,
                ClimbTempCelsius_F)
            .isOK();
    // inputs.ClimbBackMotorConnected =
    // BaseStatusSignal.refreshAll(
    //     ClimbPosition_B,
    //     ClimbVelocity_B,
    //     ClimbAppliedVolts_B,
    //     ClimbSupplyCurrent_B,
    //     ClimbTorqueCurrent_B,
    //     Climbacceleration_B,
    //     ClimbTempCelsius_B)
    // .isOK();

    // System.out.println("Arm Position: " + armPosition.getValueAsDouble());

    inputs.ClimbPositionRads = ClimbPosition_F.getValueAsDouble() * 2 * Math.PI;
    inputs.ClimbOmegaRadPerSec = ClimbVelocity_F.getValueAsDouble() * 2 * Math.PI;
    inputs.ClimbAppliedVolts = ClimbAppliedVolts_F.getValueAsDouble();
    inputs.ClimbSupplyCurrentAmps = ClimbSupplyCurrent_F.getValueAsDouble();
    inputs.ClimbTorqueCurrentAmps = ClimbTorqueCurrent_F.getValueAsDouble();
    inputs.ClimbTempCelsius = ClimbTempCelsius_F.getValueAsDouble();
    inputs.ClimbAlphaRadsPerSecSquared = Climbacceleration_F.getValueAsDouble() * 2 * Math.PI;

    // inputs.ClimbPositionRads = ClimbPosition_B.getValueAsDouble()*2*Math.PI;
    // inputs.ClimbOmegaRadPerSec = ClimbVelocity_B.getValueAsDouble()*2*Math.PI;
    // inputs.ClimbAppliedVolts = ClimbAppliedVolts_B.getValueAsDouble();
    // inputs.ClimbSupplyCurrentAmps = ClimbSupplyCurrent_B.getValueAsDouble();
    // inputs.ClimbTorqueCurrentAmps = ClimbTorqueCurrent_B.getValueAsDouble();
    // inputs.ClimbTempCelsius = ClimbTempCelsius_B.getValueAsDouble();
    // inputs.ClimbAlphaRadsPerSecSquared = Climbacceleration_B.getValueAsDouble()*2*Math.PI;

  }

  @Override
  public void runVolts(double volts) {
    ClimbForwardMotor.setControl(voltageControl.withOutput(volts));
    // ClimbBackMotor.setControl(new Follower(ClimbConstants.ClimbForwardMotorid, false));
  }

  @Override
  public void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    ClimbForwardMotor.setControl(
        positionControl
            .withPosition(angleRads / (2 * Math.PI))
            .withVelocity(omegaRadPerSec / (2 * Math.PI)));
    // ClimbBackMotor.setControl(new Follower(ClimbConstants.ClimbForwardMotorid, false));
  }

  // @Override
  // public void runMotionMagicClimbPosition(double angleRads, double omegaRadPerSec, double
  // alphaRadsPerSecSquared, double currentPositionRads) {
  //     ClimbForwardMotor.setControl(positionControl
  //         .withPosition(angleRads / (2 * Math.PI))
  //         .withVelocity(omegaRadPerSec / (2 * Math.PI))
  //         .withFeedForward(ClimbConstants.Kg*Math.cos(currentPositionRads)));
  //     ClimbBackMotor.setControl(new Follower(ClimbConstants.ClimbForwardMotorid, false));
  // }

  @Override
  public void runMotionMagicClimbPosition(double angleRads) {
    motionmagicControl.Position = angleRads / (2 * Math.PI);
    ClimbForwardMotor.setControl(motionmagicControl);
    // ClimbBackMotor.setControl(follower);
  }

  // @Override
  // public void runMotionMagicClimbPosition(double angleRads) {
  //     motionmagicControl.Position = angleRads/(2*Math.PI);
  //     ClimbForwardMotor.setControl(motionmagicControl);
  //     ClimbBackMotor.setControl(new Follower(ClimbConstants.ClimbForwardMotorid, false));
  // }

  @Override
  public void setClimbPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {
    ClimbFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    ClimbFXConfiguration.Slot0.kP = kP;
    ClimbFXConfiguration.Slot0.kI = kI;
    ClimbFXConfiguration.Slot0.kD = kD;
    ClimbFXConfiguration.Slot0.kS = kS;
    ClimbFXConfiguration.Slot0.kV = kV;
    ClimbFXConfiguration.Slot0.kA = kA;
    ClimbFXConfiguration.MotionMagic.MotionMagicAcceleration = MotionMagicAcceleration;
    ClimbFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    ClimbForwardMotor.getConfigurator().apply(ClimbFXConfiguration, 0.01);
    // ClimbBackMotor.getConfigurator().apply(ClimbFXConfiguration, 0.01);
  }

  @Override
  public void resetPosition(double angleRads) {
    angleRads = angleRads / (2 * Math.PI);
    ClimbForwardMotor.getConfigurator().setPosition(angleRads);
  }

  @Override
  public void setPIDMode(boolean enable) {
    setClimbPID(
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

  @Override
  public void stop() {
    ClimbForwardMotor.setControl(neutralControl);
    // ClimbBackMotor.setControl(neutralControl);
  }

  @Override
  public void switchBrake(boolean isBrake) {
    if (isBrake) {
      ClimbFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      ClimbFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
    ClimbForwardMotor.getConfigurator().apply(ClimbFXConfiguration, 0.01);
  }

  // @Override
  // public void runMotionMagicClimbPosition(double goal, double angleRads) {
  //     // TODO Auto-generated method stub
  //     throw new UnsupportedOperationException("Unimplemented method
  // 'runMotionMagicClimbPosition'");
  // }

}
