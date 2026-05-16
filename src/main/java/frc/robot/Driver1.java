package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class Driver1 {

  private static int aButton;

  public final int translationAxis = 1;
  public final int strafeAxis = 0;
  public final int rotationAxis = 0;

  public static final PS5Controller leftController = new PS5Controller(0);
  public static final PS5Controller rightController = new PS5Controller(1);

  public static final JoystickButton zeroHeading = new JoystickButton(rightController, 13);

  // public static final JoystickButton IntakeArmHome = new JoystickButton(leftController, 3);
  public static final JoystickButton LiftIntakeArm = new JoystickButton(leftController, 4);
  public static final JoystickButton HeadZero = new JoystickButton(rightController, 3);
  public static final JoystickButton RevertShooter = new JoystickButton(leftController, 3);
  public static final JoystickButton AimTarget = new JoystickButton(rightController, 2);
  public static final JoystickButton Driver1AimHub = new JoystickButton(leftController, 2);
  public static final JoystickButton BackShooting = new JoystickButton(rightController, 2);
  public static final JoystickButton LeftTrigger = new JoystickButton(leftController, 1);
  public static final JoystickButton RightTrigger = new JoystickButton(rightController, 1);

  public static final JoystickButton NextSong = new JoystickButton(rightController, 8);
  public static final JoystickButton PlayMusic1 = new JoystickButton(leftController, 17);
  public static final JoystickButton AimSwitch = new JoystickButton(rightController, 17);
  public static final JoystickButton PlayMusic3 = new JoystickButton(rightController, 16);
  public static final JoystickButton PlayMusic4 = new JoystickButton(rightController, 14);
  public static final JoystickButton PlayMusic5 = new JoystickButton(leftController, 8);
  public static final JoystickButton PlayMusic6 = new JoystickButton(leftController, 10);
  public static final JoystickButton StopMusic = new JoystickButton(rightController, 11);
  // public static final POVButton GroundIntake = new POVButton(leftController, 180);

  public static final JoystickButton ReportData = new JoystickButton(rightController, 5);
  public static final JoystickButton RecordData = new JoystickButton(rightController, 6);

  public static final POVButton Climb = new POVButton(rightController, 0);
  public static final POVButton ClimbDown = new POVButton(rightController, 180);

  public static final POVButton Warmup = new POVButton(leftController, 0);
  public static final POVButton SmashBumpLeft = new POVButton(leftController, 270);
  public static final POVButton SmashBumpRight = new POVButton(leftController, 90);

  public static final JoystickButton ToPose = new JoystickButton(leftController, 14);
  public static final JoystickButton TestClimbAlign = new JoystickButton(leftController, 15);
  public static final JoystickButton ApplyDeadBand = new JoystickButton(leftController, 0);

  public static final JoystickButton IntakeArmHome = new JoystickButton(leftController, 5);

  // public static final JoystickButton IntakeArmPark = new JoystickButton(leftController, 2);
  public static final JoystickButton IntakeArmPark = new JoystickButton(rightController, 4);
}
