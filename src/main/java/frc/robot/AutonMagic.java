
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.AmpScore;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;

public class AutonMagic {

    private final SendableChooser<Command> chooser;
    private Systems systems; 
    private Manipulator intake;
    private Angler pivot;
   
    public AutonMagic(Systems systems) {
        this.intake = systems.getIntake();
        this.pivot = systems.getPivot();     


        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
    }

 
    public Command procureAuton() {
        return chooser.getSelected();
    }

}
