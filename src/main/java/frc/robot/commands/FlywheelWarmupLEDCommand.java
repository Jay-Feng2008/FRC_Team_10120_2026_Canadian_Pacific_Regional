// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;

/**
 * Shows the flywheel warmup morph (SkyBlue -> OrangeRed over 1s) on the LED strip. The morph runs
 * once, always completes the full 1 second, then holds at orange until the trigger is released.
 */
public class FlywheelWarmupLEDCommand extends Command {
  private final LED led;

  public FlywheelWarmupLEDCommand(LED led) {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.setFlywheelWarmupActive(true, Timer.getFPGATimestamp());
  }

  @Override
  public void end(boolean interrupted) {
    led.setFlywheelWarmupActive(false, 0.0);
  }
}
