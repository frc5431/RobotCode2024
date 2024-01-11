package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    public Modes mode;

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

    public void setMode (Modes mode) {
        this.mode = mode;
        SmartDashboard.putString("Mode is: ", mode.toString());
    }

    public enum Modes {
        INTAKE,
        OUTTAKE,
        STOPPED,
        NEUTRAL
    }
}
