// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class RollerSystemIOReal implements RollerSystemIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final VoltageOut voltageControl = new VoltageOut(0);

  public RollerSystemIOReal(int Motorid, boolean inverted, double reduction) {

    if (reduction == 1) {
      talon = new TalonFX(Motorid, "canivore");
    } else {
      talon = new TalonFX(Motorid);
    }

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Set feedback gearing so the Talon reports in motor rotations
    // (We handle the reduction manually in updateInputs to keep it clean)
    config.Feedback.SensorToMechanismRatio = reduction;
    /*Torque Current Limiting */
    config.TorqueCurrent.PeakForwardTorqueCurrent = 200.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -200.0;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = 120.0;
    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    // talon.getConfigurator().apply(config);
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    // Optimize CAN bus usage by syncing signals
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
    PhoenixUtil.registerStatusSignals(
        Hertz.of(50.0),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    // Refresh signals to get the latest data from the CAN bus
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
            .isOK();
    // Phoenix 6 returns Rotations; convert to Radians
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void applyOutputs(RollerSystemIOOutputs outputs) {
    talon.setControl(voltageControl.withOutput(outputs.appliedVoltage));
  }
}
