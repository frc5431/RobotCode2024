package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.AmperConstants;

public class Amper extends SubsystemBase {

  public CANSparkBase motor;
  private DigitalInput beamBreak;

  protected boolean containedGamePiece;

  public RelativeEncoder upperRelativeEncoder;

  private IntakeModes mode;

  public Amper(CANSparkBase motor) {
    this.motor = motor;
    this.mode = IntakeModes.STOPPED;
    this.beamBreak = new DigitalInput(8);

    motor.setInverted(false);

    this.upperRelativeEncoder = motor.getEncoder();

    this.motor.burnFlash();

  }

  public void runNeutral() {
    this.motor.set(0);
  }

  public double getAverageAppliedOutput() {
    return motor.getAppliedOutput() / 2;
  }
  
  public DigitalInput getBeamBreakStatus() {
    return beamBreak;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Amper Rpm", this.upperRelativeEncoder.getVelocity());
    SmartDashboard.putString(getName() + " mode", this.mode.toString());
    SmartDashboard.putBoolean("beambreak", beamBreak.get());
  }

  public double[] getRPM() {
    return new double[] { upperRelativeEncoder.getVelocity()};
  }

  public boolean checkGamePieceStatus() {
    return beamBreak.get();
  }

  public void run(IntakeModes mode) {
    this.mode = mode;
    if (mode == IntakeModes.OUTAKE) {
      runWithPower(AmperConstants.outtakePower);
    } else if (mode == IntakeModes.INTAKE) {
      runWithPower(AmperConstants.intakePower);
    } else if (mode == IntakeModes.STOPPED) {
      runWithPower(0);
    }
  }

  public void runWithPower(double power) {
    motor.set(power);
  }

  public Command runPower(double power) {
    return new StartEndCommand(() -> runWithPower(power), () -> runWithPower(0), this);
  }

  public Command runMode(IntakeModes mode) {
    return new StartEndCommand(() -> run(mode), () -> runWithPower(0), this);
  }

}