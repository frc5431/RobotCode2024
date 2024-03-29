package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ManipulatorConstants;

public class Intake extends SubsystemBase {

  public CANSparkBase upper;
  public CANSparkBase lower;
  private DigitalInput beamBreak;

  protected boolean containedGamePiece;
  protected MotorRatio ratio;

  public RelativeEncoder upperRelativeEncoder;
  public RelativeEncoder lowerRelativeEncoder;
  private ManipulatorConstants constants;

  private IntakeModes mode;

  public Intake(CANSparkBase upper, CANSparkBase lower, ManipulatorConstants constants) {
    this.upper = upper;
    this.lower = lower;
    this.mode = IntakeModes.STOPPED;
    this.constants = constants;
    this.beamBreak = new DigitalInput(9);

    upper.setInverted(constants.isInverted);
    lower.setInverted(constants.isInverted);
    this.upper.burnFlash();
    this.lower.burnFlash();

    this.upperRelativeEncoder = upper.getEncoder();
    this.lowerRelativeEncoder = lower.getEncoder();

    this.ratio = constants.defaultRatio;

  }

  public ManipulatorConstants geConstants() {
    return constants;
  }

  public void runNeutral() {
    this.upper.set(0);;
    this.lower.set(0);
  }

  public double getAverageAppliedOutput() {
    return (upper.getAppliedOutput() + lower.getAppliedOutput()) / 2;
  }
  
  public DigitalInput getBeamBreakStatus() {
    return beamBreak;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Rpm", this.upperRelativeEncoder.getVelocity());
    SmartDashboard.putString(getName() + " mode", this.mode.toString());
    SmartDashboard.putBoolean("beambreak", beamBreak.get());
  }

  public double[] getRPM() {
    return new double[] { upperRelativeEncoder.getVelocity(), lowerRelativeEncoder.getVelocity() };
  }

  public boolean checkGamePieceStatus() {
    return beamBreak.get();
  }

  public void run(IntakeModes mode) {
    this.mode = mode;
    if (mode == IntakeModes.OUTAKE) {
      runWithPower(constants.outakeSpeed);
    } else if (mode == IntakeModes.INTAKE) {
      runWithPower(constants.intakeSpeed);
    } else if (mode == IntakeModes.STOPPED) {
      runWithPower(0);
    }
  }

  public void runWithPower(double power) {
    upper.set(power);
    lower.set(power);
  }

  public Command runPower(double power) {
    return new StartEndCommand(() -> runWithPower(power), () -> runWithPower(0), this);
  }

  public Command runMode(IntakeModes mode) {
    return new StartEndCommand(() -> run(mode), () -> runWithPower(0), this);
  }

  public void setRatio(MotorRatio ratio) {
    this.ratio = ratio;
  }

  /**
   * @param upperPercent a percentage in [0,1]
   * @param upperPercent a percentage in [0,1]
   */
  public record MotorRatio(double upperPercent, double lowerPercent) {
  }


}