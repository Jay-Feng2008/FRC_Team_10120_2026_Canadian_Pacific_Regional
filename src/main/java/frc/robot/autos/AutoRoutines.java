// package frc.robot.autos;

// import java.util.HashMap;

// import com.google.flatbuffers.FlexBuffers.Map;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.rollers.RollerSystem;

// public class AutoRoutines {
//     private final Drive drive;
//     private final Intake intake;
//     public AutoRoutines(Drive drive, Intake intake) {
//         this.drive = drive;
//         this.intake = intake;

//     }

// //     public Command buildAuto() {
// //     // 多段路径
// //     PathPlannerPath path1 = loadPath("StartToIntake");
// //     PathPlannerPath path2 = loadPath("IntakeToShoot");
// //     PathPlannerPath path3 = loadPath("ShootToExit");

// //     // 如果红队，翻转所有路径
// //     if (getAlliance() == DriverStation.Alliance.Red) {
// //       path1 = path1.mirrorPath();
// //       path2 = path2.mirrorPath();
// //       path3 = path3.mirrorPath();
// //     }

// //     // marker 对应动作
// //     Map<String, Runnable> markers = new HashMap<>();
// //     markers.put("intakeOn", () -> intake.setGoal(Intake.Goal.INTAKE));
// //     markers.put("intakeOff", () -> intake.setGoal(Intake.Goal.STOP));
// //     // 可以继续加 shooter、hood 等动作

// //     PathPlannerAutoBuilder autoBuilder =
// //         new PathPlannerAutoBuilder(
// //             drive::resetOdometry,
// //             drive::getPose,
// //             drive::runVelocity,
// //             drive,
// //             markers);

// //     // 多段路径组合成一个命令序列
// //     return autoBuilder.fullAuto(path1, path2, path3);
// //   }

//   /** 从文件加载路径，异常处理封装 */
//   private PathPlannerPath loadPath(String name) {
//     try {
//       return PathPlannerPath.fromPathFile(name);
//     } catch (Exception e) {
//       DriverStation.reportError("Failed to load path: " + name, e.getStackTrace());
//       return null;
//     }
//   }

//   /** 获取联盟 */
//   private DriverStation.Alliance getAlliance() {
//     try {
//       return DriverStation.getAlliance().get();
//     } catch (Exception e) {
//       return DriverStation.Alliance.Blue;
//     }
//   }
// }

//     // public AutoRoutine test1PathAuto() {
//     //     final AutoRoutine routine = m_factory.newRoutine("Test");
//     //     final AutoTrajectory simplePath = routine.trajectory("test");
//     //     routine.active().onTrue(
//     //         simplePath.resetOdometry()
//     //             .andThen(simplePath.cmd())
//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine test2() {
//     //     final AutoRoutine routine = m_factory.newRoutine("Test2");
//     //     final AutoTrajectory simplePath = routine.trajectory("test2");

//     //     routine.active().onTrue(
//     //         simplePath.resetOdometry()
//     //             .andThen(simplePath.cmd())
//     //             // .andThen(()->m_upper.setElevatorState(ElevatorState.ScoreL2))
//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine test() {
//     //     final AutoRoutine routine = m_factory.newRoutine("S2C4l");
//     //     final AutoTrajectory simplePath = routine.trajectory("S2C4l");

//     //     routine.active().onTrue(
//     //         simplePath.resetOdometry()
//     //             .andThen(simplePath.cmd())
//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine AutoS2C4lSTr() {
//     //     final AutoRoutine routine = m_factory.newRoutine("S2C4lSTr");
//     //     final AutoTrajectory simplePath_S2C4lp1 = routine.trajectory("S2C4lp1");
//     //     final AutoTrajectory simplePath_S2C4lp2 = routine.trajectory("S2C4lp2");
//     //     final AutoTrajectory simplePath_C4lSTr = routine.trajectory("C4lSTr");

//     //     routine.active().onTrue(
//     //         simplePath_S2C4lp1.resetOdometry()
//     //             .andThen(simplePath_S2C4lp1.cmd())
//     //             .andThen(()->m_upper.setElevatorState(ElevatorState.ScoreL4))
//     //             .andThen(new WaitCommand(4))
//     //             .andThen(simplePath_S2C4lp2.cmd())
//     //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //             .andThen(new WaitCommand(2))
//     //             .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //             // .andThen(simplePath_C4lSTr.cmd())
//     //     );
//     //     return routine;
//     // }

//     public AutoRoutine test2026_123() {
//         final AutoRoutine routine = m_factory.newRoutine("test2026_123");
//         final AutoTrajectory test2026_1 = routine.trajectory("test2026_1");
//         // final AutoTrajectory simplePath_S2C4lp2 = routine.trajectory("S2C4lp2");
//         final AutoTrajectory test2026_2 = routine.trajectory("test2026_2");

//         routine.active().onTrue(
//             test2026_1.resetOdometry()
//                 .andThen(test2026_1.cmd())
//                 .andThen(new WaitCommand(2))
//                 .andThen(()->intake.setGoal(Intake.Goal.INTAKE))
//                 .andThen(new WaitCommand(0.5))
//                 .andThen(()->intake.setGoal(Intake.Goal.OUTTAKE))
//         );
//         return routine;
//     }
// }
//     // public AutoRoutine S2C4lS() {
//     //     final AutoRoutine routine = m_factory.newRoutine("S2C4l+S");
//     //     final AutoTrajectory simplePath_S2C4l = routine.trajectory("S2C4l");
//     //     // final AutoTrajectory simplePath_S2C4lp2 = routine.trajectory("S2C4lp2");
//     //     final AutoTrajectory simplePath_C4lSTr = routine.trajectory("C4lSTr");

//     //     routine.active().onTrue(
//     //         simplePath_S2C4l.resetOdometry()
//     //             .andThen(simplePath_S2C4l.cmd())
//     //             .andThen(new WaitCommand(2))
//     //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //             .andThen(new WaitCommand(0.5))
//     //             .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //             .andThen(simplePath_C4lSTr.cmd())
//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine C4lSTr() {
//     //     final AutoRoutine routine = m_factory.newRoutine("C4lSTr");
//     //     final AutoTrajectory simplePath_C4lSTr = routine.trajectory("C4lSTr");

//     //     routine.active().onTrue(
//     //         simplePath_C4lSTr.resetOdometry()
//     //             .andThen(simplePath_C4lSTr.cmd())
//     //             // .andThen(()->m_upper.setElevatorState(ElevatorState.ScoreL2))
//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine S1C5rSTlC6r() {    //2个L4 稳   station
//     //     final AutoRoutine routine = m_factory.newRoutine("S1C5rSTlC6r");
//     //     final AutoTrajectory simplePath_S2C5r = routine.trajectory("S1C5r");
//     //     final AutoTrajectory simplePath_C5rSTl = routine.trajectory("C5rSTl");
//     //     final AutoTrajectory simplePath_STlC6r = routine.trajectory("STlC6r");
//     //     final AutoTrajectory simplePath_C6rSTl = routine.trajectory("C6rSTl");
//     //     final AutoTrajectory simplePath_C6rSTl_L3 = routine.trajectory("STlC6r_L3");
//     //     // final AutoTrajectory simplePath_S2C5lp1 = routine.trajectory("S1C5rp1");

//     //     routine.active().onTrue(
//     //         simplePath_S2C5r.resetOdometry()
//     //         .andThen( simplePath_S2C5r.cmd())
//     //         .andThen(new WaitCommand(0.5))   //0.5
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(simplePath_C5rSTl.cmd())
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //         .andThen(new WaitCommand(0.6))   //余量大
//     //         .andThen(simplePath_STlC6r.cmd())
//     //         .andThen(new WaitCommand(0.6))
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.5))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(simplePath_C6rSTl.cmd())
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //         .andThen(simplePath_C6rSTl_L3.cmd())

//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine S3C3lSTrC2r() {        //13s  //2个L4 稳
//     //     final AutoRoutine routine = m_factory.newRoutine("S3C3lSTrC2r");
//     //     final AutoTrajectory simplePath_S3C3l = routine.trajectory("S3C3l");
//     //     final AutoTrajectory simplePath_C3lSTr = routine.trajectory("C3lSTr");
//     //     final AutoTrajectory simplePath_STrC2r = routine.trajectory("STrC2r");
//     //     final AutoTrajectory simplePath_C2rSTr = routine.trajectory("C2rSTr");
//     //     final AutoTrajectory simplePath_STrC2l = routine.trajectory("STrC2l");

//     //     routine.active().onTrue(
//     //         simplePath_S3C3l.resetOdometry()
//     //             .andThen(simplePath_S3C3l.cmd())
//     //             .andThen(new WaitCommand(0.5))
//     //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //             .andThen(new WaitCommand(0.5))
//     //             .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //             .andThen(simplePath_C3lSTr.cmd())
//     //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //             .andThen(new WaitCommand(0.6))
//     //             .andThen(simplePath_STrC2r.cmd())
//     //             .andThen(new WaitCommand(0.3))
//     //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //             .andThen(new WaitCommand(0.3))
//     //             .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //             .andThen(new WaitCommand(0.3))
//     //             .andThen(simplePath_C2rSTr.cmd())
//     //             .andThen(new WaitCommand(0.3))
//     //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //             .andThen(simplePath_STrC2l.cmd())

//     //     );
//     //     return routine;
//     // }

//     // public AutoRoutine S1C5rSTlC6lSTlC6r() {
//     //     final AutoRoutine routine = m_factory.newRoutine("S1C5rSTlC6lSTlC6r");
//     //     final AutoTrajectory simplePath_S1C5rL3 = routine.trajectory("S1C5r_fast");
//     //     final AutoTrajectory simplePath_C5rSTl = routine.trajectory("C5rSTl_fast");
//     //     final AutoTrajectory simplePath_STlC6rL3 = routine.trajectory("STlC6r_fast");
//     //     final AutoTrajectory simplePath_C6rSTl = routine.trajectory("C6rSTl_fast");
//     //     final AutoTrajectory simplePath_STlC6lL3 = routine.trajectory("STlC6l_fast");
//     //     // final AutoTrajectory simplePath_S2C5lp1 = routine.trajectory("S1C5rp1");

//     //     routine.active().onTrue(
//     //         simplePath_S1C5rL3.resetOdometry()
//     //         .andThen( simplePath_S1C5rL3.cmd())//2.4
//     //         .andThen(new WaitCommand(0.3))//0.3
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.4))//0.4
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(simplePath_C5rSTl.cmd())//2.5
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //         .andThen(new WaitCommand(0.8))//
//     //         .andThen(simplePath_STlC6rL3.cmd())//2.2
//     //         .andThen(new WaitCommand(0.3))//0.3
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.4))//0.3
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(simplePath_C6rSTl.cmd())//2.4
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //         .andThen(new WaitCommand(0.8))
//     //         .andThen(simplePath_STlC6lL3.cmd())//2.4
//     //         .andThen(new WaitCommand(0.3))//0.2
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.4))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))

//     //     );
//     //     return routine;

//     // }

//     // public AutoRoutine S1C5rSTlC6lSTlC6r() {
//     //     final AutoRoutine routine = m_factory.newRoutine("S1C5rSTlC6lSTlC6r");
//     //     final AutoTrajectory simplePath_S1C5rL3 = routine.trajectory("S1C5r_L3");
//     //     final AutoTrajectory simplePath_C5rSTl = routine.trajectory("C5rSTl");
//     //     final AutoTrajectory simplePath_STlC6rL3 = routine.trajectory("STlC6r_L3");
//     //     final AutoTrajectory simplePath_C6rSTl = routine.trajectory("C6rSTl");
//     //     final AutoTrajectory simplePath_STlC6lL3 = routine.trajectory("STlC6l_L3");
//     //     // final AutoTrajectory simplePath_S2C5lp1 = routine.trajectory("S1C5rp1");

//     //     routine.active().onTrue(
//     //         simplePath_S1C5rL3.resetOdometry()
//     //         .andThen( simplePath_S1C5rL3.cmd())
//     //         .andThen(new WaitCommand(0.5))
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(simplePath_C5rSTl.cmd())
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //         .andThen(new WaitCommand(1))
//     //         .andThen(simplePath_STlC6rL3.cmd())
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
//     //         .andThen(simplePath_C6rSTl.cmd())
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
//     //         .andThen(new WaitCommand(1))
//     //         .andThen(simplePath_STlC6lL3.cmd())
//     //         .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
//     //         .andThen(new WaitCommand(0.3))
//     //         .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))

//     //     );
//     //     return routine;

//     // }

// //     public AutoRoutine S3C3lSTrC2rSTrC2l() {
// //         final AutoRoutine routine = m_factory.newRoutine("S3C3lSTrC2r");
// //         final AutoTrajectory simplePath_S3C3lL3 = routine.trajectory("S3C3l_fast");
// //         final AutoTrajectory simplePath_C3lSTr = routine.trajectory("C3lSTr_fast2");
// //         final AutoTrajectory simplePath_STrC2rL3 = routine.trajectory("STrC2r_fast");
// //         final AutoTrajectory simplePath_C2rSTr = routine.trajectory("C2rSTr_fast");
// //         final AutoTrajectory simplePath_STrC2lL3 = routine.trajectory("STrC2l_fast");
// //         // final AutoTrajectory simplePath_C2lSTr = routine.trajectory("C2lSTr");

// //         routine.active().onTrue(
// //             simplePath_S3C3lL3.resetOdometry()
// //                 .andThen(simplePath_S3C3lL3.cmd())//2.5
// //                 .andThen(new WaitCommand(0.4))//0.4
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //                 .andThen(new WaitCommand(0.3))//0.3
// //                 .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //                 .andThen(simplePath_C3lSTr.cmd())//1.8
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
// //                 .andThen(new WaitCommand(0.9))
// //                 .andThen(simplePath_STrC2rL3.cmd())//2.3
// //                 .andThen(new WaitCommand(0.3))//0.3
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //                 .andThen(new WaitCommand(0.4))//0.4
// //                 .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //                 .andThen(simplePath_C2rSTr.cmd())//2.5
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))
// //                 .andThen(new WaitCommand(0.8))
// //                 .andThen(simplePath_STrC2lL3.cmd())//2.3
// //                 .andThen(new WaitCommand(0.3))//0.3
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //                 .andThen(new WaitCommand(0.5))//0.5
// //                 .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //         );
// //         return routine;
// //     }

// //     public AutoRoutine PushS1() {
// //         final AutoRoutine routine = m_factory.newRoutine("PushS1");
// //         final AutoTrajectory simplePath_S2C5r = routine.trajectory("S1C5r");
// //         final AutoTrajectory simplePath_XC5lPush2 = routine.trajectory("X-C5lPush2");
// //         // final AutoTrajectory simplePath_STlC6r = routine.trajectory("STlC6r");
// //         // final AutoTrajectory simplePath_S2C5lp1 = routine.trajectory("S1C5rp1");

// //         routine.active().onTrue(
// //             simplePath_S2C5r.resetOdometry()
// //             .andThen( simplePath_S2C5r.cmd())
// //             .andThen(new WaitCommand(0.5))
// //             .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //             .andThen(new WaitCommand(0.3))
// //             .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //             .andThen(simplePath_XC5lPush2.cmd())
// //             // .andThen(()->m_upper.setIntakeState(IntakeState.IntakeCapture))

// //         );
// //         return routine;
// //     }

// //     public AutoRoutine PushS3() {
// //         final AutoRoutine routine = m_factory.newRoutine("PushS3");
// //         final AutoTrajectory simplePath_S3C3l = routine.trajectory("S3C3l");
// //         final AutoTrajectory simplePath_XC3lPush3 = routine.trajectory("X-C3lPush3");
// //         // final AutoTrajectory simplePath_STrC2r = routine.trajectory("STrC2r");

// //         routine.active().onTrue(
// //             simplePath_S3C3l.resetOdometry()
// //                 .andThen(simplePath_S3C3l.cmd())
// //                 .andThen(new WaitCommand(0.8))
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //                 .andThen(new WaitCommand(0.5))
// //                 .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //                 .andThen(simplePath_XC3lPush3.cmd())

// //         );
// //         return routine;
// //     }

// //     public AutoRoutine AutoS3C3lSTrC2lSTrC2r() {
// //         final AutoRoutine routine = m_factory.newRoutine("S3C3lSTr");
// //         final AutoTrajectory simplePath_S3C3l = routine.trajectory("S3C3l");
// //         final AutoTrajectory simplePath_C3lSTr = routine.trajectory("C3lSTr");
// //         final AutoTrajectory simplePath_STrC2l = routine.trajectory("STrC2l");
// //         final AutoTrajectory simplePath_C2lSTr = routine.trajectory("C2lSTr");
// //         final AutoTrajectory simplePath_STrC2r = routine.trajectory("STrC2r");

// //         routine.active().onTrue(
// //             simplePath_S3C3l.resetOdometry()
// //                 .andThen(simplePath_S3C3l.cmd())
// //                 // .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScoring))
// //                 .andThen(new WaitCommand(2))
// //                 .andThen(simplePath_C3lSTr.cmd())
// //                 // .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //                 // .andThen(new WaitCommand(5))

// //         );
// //         return routine;
// //     }

// //     public AutoRoutine AutoS3C3lSTrC2l() {
// //         final AutoRoutine routine = m_factory.newRoutine("S3C3lSTr");
// //         final AutoTrajectory simplePath_S3C3l = routine.trajectory("S3C3l");
// //         final AutoTrajectory simplePath_C3lSTr = routine.trajectory("C3lSTr");
// //         final AutoTrajectory simplePath_STrC2l = routine.trajectory("STrC2l");

// //         routine.active().onTrue(
// //             simplePath_S3C3l.resetOdometry()
// //                 .andThen(simplePath_S3C3l.cmd())
// //                 // .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScore))
// //                 .andThen(()->m_upper.setIntakeState(IntakeState.IntakeScoring))
// //                 .andThen(new WaitCommand(1))
// //                 .andThen(simplePath_C3lSTr.cmd())
// //                 .andThen(new WaitCommand(2))
// //                 .andThen(simplePath_STrC2l.cmd())
// //                 // .andThen(()->m_upper.setElevatorState(ElevatorState.StationIntake))
// //                 // .andThen(new WaitCommand(5))

// //         );
// //         return routine;
// //     }

// // }
