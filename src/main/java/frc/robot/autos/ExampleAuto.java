// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.intake.Intake;
// import com.pathplanner.lib.path.PathPlannerPath;
// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;

// public class ExampleAuto {

//     public static Command buildAuto(Drive drive, Intake intake) {
//        // Use the PathPlannerAuto class to get a path group from an auto
//        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example
// Auto");

//         // 2️⃣ 定义事件 map（GUI 中 Event Markers 名字 → Command）
//         Map<String, Command> eventMap = new HashMap<>();
//         eventMap.put("intakeOn", Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE),
// intake));
//         eventMap.put("intakeOff", Commands.runOnce(() -> intake.setGoal(Intake.Goal.STOP),
// intake));
//         // 可以继续添加其他事件，比如 shooter.spinUpCommand()

//         // 3️⃣ 构建 AutoBuilder
//         PPAutoBuilder autoBuilder = new PPAutoBuilder(
//                 drive::getPose,         // Pose supplier
//                 drive::resetOdometry,   // Reset odometry at start of path
//                 drive.getKinematics(),  // 机械学
//                 drive.getXController(),
//                 drive.getYController(),
//                 drive.getThetaController(),
//                 drive::setModuleStates, // 将输出给底盘
//                 eventMap,               // 事件 map
//                 true                    // 是否自动 reset odometry
//         );

//         // 4️⃣ 构建最终命令
//         return autoBuilder.fullAuto(trajectories);
//     }
// }
