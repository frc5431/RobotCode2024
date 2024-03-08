package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RunClimberCommand extends Command {
    private Climber climber;

    public enum ClimberMode {
        EXTENDED,
        RETRACTED
    }

    private ClimberMode mode;

    public RunClimberCommand(Climber climber, ClimberMode mode) {
        this.climber = climber;
        this.mode = mode;
        
    }

    @Override
    public void initialize() {
        double position;
        if(mode == ClimberMode.EXTENDED) {
            position = Constants.ClimberConstants.minHeight;
        }else if(mode == ClimberMode.RETRACTED) {
            position = Constants.ClimberConstants.maxHeight;
        }else {
            throw new IllegalStateException();
        }

        this.climber.setPosition(position);
    }
}
