
// package frc.robot;

// import java.util.List;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutonMagic {

// private final SendableChooser<Command> chooser = new SendableChooser<>();
    
//     public AutoBuilder autonBuilder;
//     public PathConstraints constraints;
//     public String pathLocation;

//     public AutonMagic(AutoBuilder autonBuilt, String pathLocation, PathConstraints constraints) {
//         this.autonBuilder = autonBuilt;
//         this.constraints = constraints;
//         this.pathLocation = pathLocation;

//         List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(pathLocation);

//         for (PathPlannerPath pathName : pathGroup) {
//             chooser.addOption("Select the Path", getAuto(pathLocation));
//         }

//         SmartDashboard.putData("Auton", chooser);
//     }

//     public Command getAuto(String pathName) {
//         System.out.println(pathName);
//         PathPlannerPath path = (PathPlannerPath) PathPlannerPath.fromPathFile(pathName);
//         return autonBuilder.buildAuto(pathName);
//     }

//     public Command procureAuton() {
//         return chooser.getSelected();
//     }

// }
