package frc.robot.subsystems.wheelback;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class WheelbackIOReal implements WheelbackIO {
  private final TalonFX talon;
  private final TalonFX talonL;
  // 状态信号以便通过 IO 层读取
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVoltage;

  // 控制请求
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC velocityControlFOC = new VelocityTorqueCurrentFOC(0);
  private final NeutralOut coastControl = new NeutralOut();

  public WheelbackIOReal(int id, boolean isclockwice_Positive) {
    // 假设 Hood 电机 ID 为 15，位于 CANivore ("rio" 或你的 CANbus 名称)
    talon = new TalonFX(WheelbackConstants.wheelbackId);
    talonL = new TalonFX(5);

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(
                        isclockwice_Positive
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(WheelbackConstants.wheelbackGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120)) // Default
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    // .withSupplyCurrentLowerLimit(40) // Default
                    // .withSupplyCurrentLowerTime(1) // Default
                    .withSupplyCurrentLimitEnable(true));

    // .withMotionMagic(
    //     new MotionMagicConfigs()
    //         .withMotionMagicCruiseVelocity(
    //             Units.radiansToRotations(HoodConstants.velocityRadPerSec))
    //         .withMotionMagicAcceleration(
    //             Units.radiansToRotations(HoodConstants.accelerationRadPerSecSq)));

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    tryUntilOk(5, () -> talonL.getConfigurator().apply(config));
    talonL.setControl(new Follower(talon.getDeviceID(), MotorAlignmentValue.Opposed));
    // talonR.setControl(new Follower(talon.getDeviceID(), false));
    // 初始化信号
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();
    followerAppliedVoltage = talonL.getMotorVoltage();

    // 优化 CAN 总线带宽，将这些信号设为高频同步更新
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     100.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
    PhoenixUtil.registerStatusSignals(
        Hertz.of(50.0), position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(WheelbackIOInputs inputs) {
    // 刷新所有信号
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();
    inputs.followerConnected = BaseStatusSignal.refreshAll(followerAppliedVoltage).isOK();

    // CTRE 默认单位是 rotations, 转换为 WPILib 惯用的 Radians
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
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
    tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    // 将弧度/秒转回 rotations/sec 给 TalonFX
    double rps = Units.radiansToRotations(velocityRadsPerSec);
    talon.setControl(velocityControlFOC.withVelocity(rps));
  }

  @Override
  public void applyOutputs(WheelbackIOOutputs outputs) {
    switch (outputs.mode) {
      case VELOCITY:
        // Kraken 使用 Rotations/Sec，需要转换
        double rps = Units.radiansToRotations(outputs.velocityRadsPerSec);
        talon.setControl(velocityControlFOC.withVelocity(rps));
        break;

      case VOLTAGE:
        talon.setControl(voltageControl.withOutput(outputs.volts));
        break;

      case COAST:
      default:
        talon.setControl(coastControl);
        break;
    }
  }
}
