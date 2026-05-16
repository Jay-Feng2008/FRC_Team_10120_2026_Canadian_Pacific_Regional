package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Driver2 {

  private static int aButton;

  public static final PS5Controller Controller = new PS5Controller(2);

  public static final JoystickButton Park = new JoystickButton(Controller, 6);
  // public static final JoystickButton SetPoseLeft = new JoystickButton(Controller, 7);
  // public static final JoystickButton SetPoseRight = new JoystickButton(Controller, 8);

  public static final Trigger AimHub = new JoystickButton(Controller, 1);
  public static final Trigger RightButton = new JoystickButton(Controller, 3);
  public static final Trigger LiftIntakeArm = new JoystickButton(Controller, 4);
  public static final Trigger DownButton = new JoystickButton(Controller, 3);

  public static final Trigger LeftPOV = new POVButton(Controller, 270);
  public static final Trigger RightPOV = new POVButton(Controller, 90);
  public static final Trigger ClimbDown = new POVButton(Controller, 0);
  public static final Trigger Climb = new POVButton(Controller, 180);

  public static final JoystickButton Enable = new JoystickButton(Controller, 5);

  public static final JoystickButton RightTrigger = new JoystickButton(Controller, 8);

  public static final JoystickButton GroundIntake = new JoystickButton(Controller, 7);

  public final JoystickButton setTargetToLeftReef = new JoystickButton(Controller, 1);
  public final JoystickButton setTargetToRightReef = new JoystickButton(Controller, 3);
  public final POVButton coralreef = new POVButton(Controller, 0);

  public final POVButton enableTeleOP = new POVButton(Controller, 180);

  public static final JoystickButton CoastIntakeArm = new JoystickButton(Controller, 13);
}
