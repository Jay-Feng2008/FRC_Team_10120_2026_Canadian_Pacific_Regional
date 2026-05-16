package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class SimDriver {

  private static int aButton;

  public static final int translationAxis = 1;
  public static final int strafeAxis = 0;
  public static final int rotationAxis = 4;

  public static final PS5Controller controller = new PS5Controller(0);
  // public static final PS5Controller controller2 = new PS5Controller(1);

  public static final JoystickButton zeroHeading = new JoystickButton(controller, 1); // Button X
  public static final JoystickButton HeadZero = new JoystickButton(controller, 3); // Button ∆
  public static final JoystickButton AimTarget = new JoystickButton(controller, 4); // Button ▢
  public static final JoystickButton Warmup = new JoystickButton(controller, 2); // Button ◯

  public static final JoystickButton RightTrigger = new JoystickButton(controller, 6); // R1
  public static final JoystickButton LeftTrigger = new JoystickButton(controller, 5); // L1

  public static final JoystickButton ClimbUp = new JoystickButton(controller, 20); //  POV up
  public static final JoystickButton ClimbDown = new JoystickButton(controller, 21); // POV down

  // public static final JoystickButton ScoreL1 = new JoystickButton(leftController, 3);
  // public static final JoystickButton ScoreL2 = new JoystickButton(leftController, 4);
  // public static final JoystickButton HeadZero = new JoystickButton(rightController, 3);
  // public static final JoystickButton ScoreL4 = new JoystickButton(rightController, 4);
  // public static final JoystickButton StationIntake = new JoystickButton(rightController, 2);
  // public static final JoystickButton Intake = new JoystickButton(leftController, 1);
  // public static final JoystickButton Eject = new JoystickButton(rightController, 1);

  // public static final JoystickButton RemoveAlgae = new JoystickButton(rightController, 12);
  // // public static final POVButton GroundIntake = new POVButton(leftController, 180);

  // public static final POVButton Climb = new POVButton(rightController, 0);
  // public static final POVButton ClimbDown = new POVButton(rightController, 180);

  // public static final JoystickButton ToPose = new JoystickButton(leftController, 14);
  // public static final JoystickButton ApplyDeadBand = new JoystickButton(leftController, 0);

  // public static final JoystickButton MicroDrive = new JoystickButton(leftController, 2);
}
