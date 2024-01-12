package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    Modes enummy;
    private Spark intakeSparky;

    public void run(double power){
        intakeSparky.set(power);
    }

    public void inTake(){
        run(Constants.IntakeConstants.intakePower);
    }

    public void outTake(){
        run(Constants.IntakeConstants.outtakePower);
    }

    public void setMode (Modes enummy) {
        this.enummy = enummy;
        SmartDashboard.putString("Mode is: ", enummy.toString());
    }

    public enum Modes{
        FORWARD,
        REVERSE,
        STOPPED,
        NEUTRAL
    }
}