package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;

public class RunAnglerCommand extends Command {
    final boolean retract;
    final Angler angler;

    public RunAnglerCommand (boolean retract, Angler angler) {
        this.retract = retract;
        this.angler = angler;
    }

    @Override
    public void initialize() {
        if (retract) {
            angler.retract();
        }
        else {
            angler.deploy();
        }
    }

    @Override
    public boolean isFinished() {
        return angler.isFinished(Units.degreesToRadians(5));
    }
}
