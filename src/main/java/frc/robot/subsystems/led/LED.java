// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.shooter.UpperStructure;
import java.util.List;

public class LED extends SubsystemBase {
  // private static LEDSubsystems instance;

  // public static LEDSubsystems getInstance(){
  //   if(instance == null){
  //     instance = new LEDSubsystems();
  //   }
  //   return instance;
  // }

  // LED IO
  private AddressableLED leds = new AddressableLED(LEDConstants.LEDPort);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.length);
  private Alliance alliance;

  // Robot state tracking
  public boolean autoFinished = false;
  public boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private double autoFinishedTime = 0.0;
  private double autoTimerError = 0.0;
  public boolean teleopStatue = false;

  private Timer loaded = new Timer();
  private Joystick controller;
  private Drive drive;
  // private UpperStructure upperStructure;

  // Flywheel warmup morph state (set by FlywheelWarmupLEDCommand)
  private boolean flywheelWarmupActive = false;
  private double flywheelWarmupStartTime = 0.0;

  public LED(Drive drive) {
    // this.controller = controller;
    this.drive = drive;
    // this.upperStructure = upperStructure;

    leds.setLength(buffer.getLength());
    // leds.setData(buffer);
    leds.start();
    loaded.start();
  }

  @Override
  public void periodic() {
    // stripes2way(List.of(Color.kAzure, Color.kGreen), 5, 0.5);
    updateLED();
    SmartDashboard.putString("ledstate", ledState.toString());
    leds.setData(buffer);
  }

  public enum LEDState {
    INITIAL,
    Warmup,
    CircleMode,
    AutoAlign,
    IntakeScoring,
    IntakeCapture,
    Climb,
    BYG,
    // L2,
    // L3,
    // L4
  }

  public LEDState ledState = LEDState.INITIAL;
  // private Joystick l_driver;
  // private Joystick l_Operater;
  // private CommandSwerveDrivetrain l_drivetrain;
  // private UpperStructure l_upper;

  // IntakeCapture,
  // Climb,

  /** Called by FlywheelWarmupLEDCommand when RightTrigger starts flywheel warmup. */
  public void setFlywheelWarmupActive(boolean active, double startTime) {
    this.flywheelWarmupActive = active;
    this.flywheelWarmupStartTime = startTime;
  }

  public void updateLED() {
    if (flywheelWarmupActive) {
      showFlywheelWarmupMorph();
      return;
    }
    // if(l_upper.currentIntakeState==IntakeState.IntakeScoring
    // ||l_upper.currentIntakeState==IntakeState.IntakeScoringL4){
    //   this.ledState = LEDState.IntakeScoring;
    // }else if(l_driver.getRawButtonPressed(1)||l_driver.getRawButtonPressed(3)){
    //     this.ledState = LEDState.AutoAlign;
    // }else if(l_driver.getRawButtonPressed(2)){
    //     this.ledState = LEDState.CircleMode;
    // }else
    // if(l_upper.currentIntakeState==IntakeState.IntakeCapture&&!l_upper.intake.getSensored()){
    //   this.ledState = LEDState.IntakeCapture;
    // }else if(l_upper.currentClimberState==ClimberState.CLIMBING){
    //     this.ledState = LEDState.Climb;
    // }else if(l_driver.getRawButtonReleased(1)
    // ||l_driver.getRawButtonReleased(2)
    // ||l_driver.getRawButtonReleased(3)
    // ||l_driver.getRawButtonReleased(8)
    // ){
    //   this.ledState = LEDState.BYG;
    // }
    showLED(this.ledState);
  }

  public void showLED(LEDState state) {
    this.ledState = state;
    switch (state) {
      case INITIAL:
        try {
          alliance = DriverStation.getAlliance().get();
        } catch (Exception e) {
          alliance = DriverStation.Alliance.Red;
        }
        // if (alliance == Alliance.Blue) {
        //   solidColor(Color.kBlue);
        // } else {
        //   solidColor(Color.kRed);
        // }
        solidColor(Color.kOrangeRed);
        break;
      case Warmup:
        morph(Color.kSkyBlue, Color.kOrangeRed, 1.0);
        break;
      case IntakeScoring:
        strobe(Color.kWhite, Color.kGreen, 0.2);
        break;
      case IntakeCapture:
        solidColor(Color.kGreen);
        break;
        // case StationIntake:
        //   solidColor(Color.kOrange);
        //   break;
      case AutoAlign:
        if (alliance == Alliance.Blue) {
          strobe(Color.kBlue, Color.kBlack, 0.2);
        } else {
          strobe(Color.kRed, Color.kBlack, 0.2);
        }
        break;
      case Climb:
        // rainbow(5, 0.4);
        rainbow(50, 2);
        break;
      case CircleMode:
        if (alliance == Alliance.Blue) {
          solidColor(Color.kBlue);
        } else {
          solidColor(Color.kRed);
        }
        break;
      case BYG:
        stripes2way(List.of(Color.kAzure, Color.kGreen), 5, 0.5);
        break;
        // case L2:
        //   solidTwoColor(LEDConstants.L2percent);
        //   break;
        // case L3:
        //   solidTwoColor(LEDConstants.L3percent);
        //   break;
      default:
        stripes2way(List.of(Color.kAzure, Color.kGreen), 5, 0.5);
        break;
    }
  }

  public void stop() {
    if (alliance == Alliance.Blue) {
      breath(
          Color.kBlue, Color.kBlack, Timer.getFPGATimestamp(), LEDConstants.EnablebreathDuration);
    } else {
      breath(Color.kRed, Color.kBlack, Timer.getFPGATimestamp(), LEDConstants.EnablebreathDuration);
    }
    ;
  }

  private void solidColor(Color color) {
    // LEDPattern lp = new LEDPattern();
    if (color != null) {
      for (int i = LEDConstants.initializelength; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solidTwoColor(int percent) {
    for (int i = 0; i < percent; i++) {
      buffer.setRGB(i, 128, 0, 128);
    }
    for (int i = percent; i < LEDConstants.length; i++) {
      buffer.setRGB(i, 0, 0, 0); // black
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solidColor(on ? c1 : c2);
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < buffer.getLength(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i <= buffer.getLength()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  /** Single morph SkyBlue->OrangeRed over 1s, holds at orange. Runs once, never loops. */
  private void showFlywheelWarmupMorph() {
    double elapsed = Timer.getFPGATimestamp() - flywheelWarmupStartTime;
    if (elapsed >= 1.0) {
      solidColor(Color.kOrangeRed);
      return;
    }
    double ratio = elapsed / 1.0;
    double red = (Color.kSkyBlue.red * (1 - ratio)) + (Color.kOrangeRed.red * ratio);
    double green = (Color.kSkyBlue.green * (1 - ratio)) + (Color.kOrangeRed.green * ratio);
    double blue = (Color.kSkyBlue.blue * (1 - ratio)) + (Color.kOrangeRed.blue * ratio);
    solidColor(new Color(red, green, blue));
  }

  private void morph(Color c1, Color c2, double duration) {
    double ratio = (Timer.getFPGATimestamp() % duration) / duration;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solidColor(new Color(red, green, blue));
  }

  private void breath(Color c1, Color c2, double timestamp, double duration) {
    double x =
        ((timestamp % LEDConstants.breathDuration) / LEDConstants.breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solidColor(new Color(red, green, blue));
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < buffer.getLength(); i++) {
      x += xDiffPerLed;
      if (i >= 0) {
        double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private void stripes(List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = 0; i < buffer.getLength(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private void stripes2way(List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = 0; i < buffer.getLength() / 2; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
      buffer.setLED(buffer.getLength() - i - 1, colors.get(colorIndex));
    }
  }
}
