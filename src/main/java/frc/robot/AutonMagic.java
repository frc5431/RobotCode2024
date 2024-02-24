
package frc.robot;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PheonixDrivebase;

public class AutonMagic {

    private final SendableChooser<Command> chooser;
    private Systems systems; 
   
    public AutonMagic(Systems systems) {
        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
    }

 
    public Command procureAuton() {
        return chooser.getSelected();
    }

}
