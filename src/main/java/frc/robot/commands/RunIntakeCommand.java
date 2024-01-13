package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeCommand extends Command {
    
    public final DirectionEnum direction;
    final Intake intake;

    public enum DirectionEnum{
        INTAKE,
        OUTTAKE
    }

    public RunIntakeCommand(DirectionEnum direction, Intake intake){
        this.direction = direction;
        this.intake = intake;
    }
    
    @Override
    public void initialize() {
        if(direction == DirectionEnum.INTAKE){
            intake.intake();
            intake.setMode(Intake.Modes.FORWARD);
        }
        else if (direction == DirectionEnum.OUTTAKE) {
            intake.outtake();
            intake.setMode(Intake.Modes.REVERSE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }

    public DirectionEnum direction(String string) {
        throw new UnsupportedOperationException("Unimplemented method 'direction'");
    }
}
