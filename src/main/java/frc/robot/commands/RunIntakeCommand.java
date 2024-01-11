package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeCommand extends Command {
    enum IntakeEnum{
        FORWARD,
        REVERSE,
        STOPPED,
        NEUTRAL
    }

    IntakeEnum enummy;
    final boolean forward;
    final Intake intake;

    @Override
    public void initialize() {
        
    }

    public RunIntakeCommand(boolean forward, Intake intake){
        this.forward = forward;
        this.intake = intake;

        if(forward){
            intake.inTake();
            enummy = IntakeEnum.FORWARD;
        }
        else {
            intake.outTake();
            enummy = IntakeEnum.REVERSE;
        }
        SmartDashboard.putString("State of Intake:",enummy.toString());
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
        enummy = IntakeEnum.STOPPED;
        SmartDashboard.putString("State of Intake:",enummy.toString());
    }
}
