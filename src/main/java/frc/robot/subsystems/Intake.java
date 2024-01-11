package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake {
    
    private Spark intakeSparky;

    public void run(double power){
        intakeSparky.set(power);
    }

    public void inTake(){
        run(1);
    }

    public void outTake(){
        run(-1);
    }
}
