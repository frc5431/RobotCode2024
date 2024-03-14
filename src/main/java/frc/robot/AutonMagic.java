
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
public class AutonMagic {

    private final SendableChooser<Command> chooser;
   
    public AutonMagic(Systems systems) {   
        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public Command procureAuton() {
        return chooser.getSelected();
    }

}
