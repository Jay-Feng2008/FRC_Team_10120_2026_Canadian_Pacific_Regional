package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");
  public static final CANBus kCANivoreCANBus = new CANBus("main");

  // Talon FX IDs
  public static final int kIntakePivot = 10;
  public static final int kIntakeRollers = 11;

  public static final int kindex = 12;

  public static final int kflywheel_1 = 14;
  public static final int kturret_1 = 15;
  public static final int khood_1 = 16;
  public static final int kFeeder_1 = 17;

  public static final int kflywheel_2 = 18;
  public static final int kturret_2 = 19;
  public static final int khood_2 = 20;
  public static final int kFeeder_2 = 21;

  public static final int kHanger = 16;
}
