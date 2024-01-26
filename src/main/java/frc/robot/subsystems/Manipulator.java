package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Manipulator extends SubsystemBase {
    protected boolean wasRanNeutral;
    public enum Modes {
        FORWARD,
        BACKWARDS,
        STOPPED,
        NEUTRAL;
    }

    public abstract double getForwardVelocityMultiplier();
    public abstract double getReverseVelocityMultiplier();

    public void run(Modes mode) {
        SmartDashboard.putString(getName() + " mode", mode.toString());
        if(wasRanNeutral && mode != Modes.NEUTRAL) {
            stopNeutral();
            wasRanNeutral = false;
        }

        if(mode == Modes.FORWARD) {
            runWithPower(getForwardVelocityMultiplier());
        }else if(mode == Modes.BACKWARDS) {
            runWithPower(getReverseVelocityMultiplier());
        }else if(mode == Modes.STOPPED) {
            runWithPower(0);
        }else {
            wasRanNeutral = true;
            runNeutral();
        }
    }

    public abstract void runNeutral();
    public abstract void stopNeutral();
    public void runWithPower(double power) {
        SmartDashboard.putNumber(getName() + " power", power);
    }

    public abstract boolean checkGamePieceStatus();
}
