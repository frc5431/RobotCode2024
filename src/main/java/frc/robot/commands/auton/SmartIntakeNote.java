package frc.robot.commands.auton;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Intake;

public class SmartIntakeNote extends Command {

    private Angler pivot;
    private Intake intake;
    
    public SmartIntakeNote(Intake intake, Angler pivot) {
        this.intake = intake;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.runToMinimum();
    }
   
    @Override
    public void execute() {
        intake.run(IntakeModes.INTAKE);
    }

    @Override 
    public boolean isFinished() {
        return intake.getBeamBreakStatus().get();
    }

    @Override
    public void end(boolean interrupted) {
        intake.runPower(0);
        pivot.setRotation(IntakeConstants.mainStowAngle);

    }

}
