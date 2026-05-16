package frc.robot.subsystems.intakearm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IntakearmIOReal implements IntakearmIO {
  private final TalonFX talon;

  // 状态信号（用于高效读取）
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // 控制请求
  private final TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final CoastOut coastControl = new CoastOut();
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
  // private final MotionMagicExpoTorqueCurrentFOC motionMagicControl = new
  // MotionMagicExpoTorqueCurrentFOC(0);
  private final StaticBrake brakeControl = new StaticBrake();

  public IntakearmIOReal() {
    // 假设 Intakearm 电机 ID 为 15，位于 CANivore ("rio" 或你的 CANbus 名称)
    talon = new TalonFX(IntakearmConstants.IntakearmId, "canivore");

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IntakearmConstants.IntakearmGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60)) // Default
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLowerLimit(40) // Default
                    .withSupplyCurrentLowerTime(0.2) // Default
                    .withSupplyCurrentLimitEnable(true))
            // .withTorqueCurrent(
            //     new TorqueCurrentConfigs()
            //         .withPeakForwardTorqueCurrent(Amps.of(100))
            //         .withPeakReverseTorqueCurrent(Amps.of(100)))
            // .withSoftwareLimitSwitch(
            //     new SoftwareLimitSwitchConfigs()
            //         .withForwardSoftLimitEnable(true)
            //         .withForwardSoftLimitThreshold(
            //             Units.radiansToRotations(
            //                 IntakearmConstants.IntakearmMaxAngle)) // radian转圈数  电机
            //         .withReverseSoftLimitEnable(true)
            //         .withReverseSoftLimitThreshold(
            //             Units.radiansToRotations(IntakearmConstants.IntakearmMinAngle)))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        Units.radiansToRotations(IntakearmConstants.velocityRadPerSec))
                    .withMotionMagicAcceleration(
                        Units.radiansToRotations(IntakearmConstants.accelerationRadPerSecSq)));

    // talon.getConfigurator().apply(config);
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    resetAngle(0);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    // 默认100hz
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     100.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
    PhoenixUtil.registerStatusSignals(
        Hertz.of(100.0), position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakearmIOInputs inputs) {
    // 刷新所有信号
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();

    // Phoenix 6 默认单位是 Rotations，需要转为 Rads
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakearmIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> talon.setControl(brakeControl);
      case COAST -> talon.setControl(coastControl);
      case CLOSED_LOOP -> {
        // talon.setControl(positionControl.withPosition(outputs.positionRad / (2 * Math.PI))
        //  .withSlot(0)  ); //
        talon.setControl(
            motionMagicControl
                .withPosition(outputs.positionRad / (2 * Math.PI))
                .withSlot(0)); // 弧度转圈数（实际电机控制
      }
      case HOMING -> {
        // talon.setControl(positionControl.withPosition(outputs.positionRad / (2 * Math.PI))
        //  .withSlot(0)  ); //
        talon.setControl(voltageControl.withOutput(IntakearmConstants.homingVoltage));
      }
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    // turretFXConfiguration.Slot0.GravityType = GravityTypeValue.turret_Cosine;
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP; // A position error of 0.2 rotations results in 12 V output
    cfg.kI = kI; // 0
    cfg.kD = kD; // A velocity error of 1 rps results in 0.5 V output
    cfg.kS = kS; // static feedforward voltage
    cfg.kV = kV; // velocity feedforward voltage
    cfg.kA = kA; // acceleration feedforward voltage
    cfg.kG = kG; // gravity feedforward voltage
    cfg.GravityType = GravityTypeValue.Arm_Cosine; // TODO：
    tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }

  // @Override
  public void resetAngle(double Radius) {
    talon.getConfigurator().setPosition(Units.radiansToRotations(Radius));
  }
}
