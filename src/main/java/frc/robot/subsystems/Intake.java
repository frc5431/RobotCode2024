package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class Intake {
    public Spark intakeMotor;

    public Intake (Spark motor) {
        intakeMotor = motor;

        intakeMotor.setInverted(false);
    }

    public void intake () {
        run(Constants.IntakeConstants.intakePower);
    }
    
    public void outtake () {
        run(Constants.IntakeConstants.outtakePower);
    }

    public void run (double power) {
        intakeMotor.set(power);
    }
}
