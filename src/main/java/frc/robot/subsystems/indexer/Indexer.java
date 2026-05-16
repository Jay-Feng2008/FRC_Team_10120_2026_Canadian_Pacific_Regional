// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
// import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {
  private static final LoggedTunableNumber IndexerIntakeVolts =
      new LoggedTunableNumber("Indexer/Roller/IndexerVolts", 6.0);
  private static final LoggedTunableNumber IndexerOuttakeVolts =
      new LoggedTunableNumber("Indexer/Roller/OuttakeVolts", -6.0);

  private final RollerSystem roller;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Indexer(RollerSystemIO rollerIO) {
    this.roller = new RollerSystem("Indexer roller", "Indexer/Roller", rollerIO);
  }

  public void periodic() {

    roller.periodic();

    double rollerVolts = 0.0;

    switch (goal) {
      case INTAKE -> {
        rollerVolts = IndexerIntakeVolts.get();
      }

      case OUTTAKE -> {
        rollerVolts = IndexerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
    roller.periodic();
  }

  // @Override
  // public void periodicAfterScheduler() {
  //   roller.periodicAfterScheduler();
  // }

  public enum Goal {
    INTAKE,
    OUTTAKE,
    STOP,
    SHOOT
  }

  public void switchState() {
    if (this.goal == Goal.STOP) {
      setGoal(Goal.INTAKE);
    } else {
      setGoal(Goal.STOP);
    }
  }

  // public enum
}
