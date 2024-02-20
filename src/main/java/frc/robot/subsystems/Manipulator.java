package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

import java.util.Optional;

public class Manipulator extends SubsystemBase {

  public CANSparkBase upper;
  public CANSparkBase lower;

  protected boolean containedGamePiece;
  private long lastFiredTimestamp = -1;
  protected MotorRatio ratio;

  public RelativeEncoder upperRelativeEncoder;
  public RelativeEncoder lowerRelativeEncoder;
  private ManipulatorConstants constants;

  public Manipulator(CANSparkBase upper, CANSparkBase lower, ManipulatorConstants constants) {
    this.upper = upper;
    this.lower = lower;

    this.constants = constants;

    upper.setInverted(constants.isInverted);
    lower.setInverted(constants.isInverted);
    this.upper.burnFlash();
    this.lower.burnFlash();

    upperRelativeEncoder = upper.getEncoder();
    lowerRelativeEncoder = lower.getEncoder();

    this.ratio = constants.defaultRatio;

  }

  public void runNeutral() {
    this.upper.setIdleMode(IdleMode.kCoast);
    this.upper.burnFlash();
    this.lower.setIdleMode(IdleMode.kCoast);
    this.lower.burnFlash();
  }

  public void stopNeutral() {
    this.upper.setIdleMode(IdleMode.kBrake);
    this.upper.burnFlash();
    this.lower.setIdleMode(IdleMode.kBrake);
    this.lower.burnFlash();
  }

  public double getAverageAppliedOutput() {
    return (upper.getAppliedOutput() + lower.getAppliedOutput()) / 2;
  }

  @Override
  public void periodic() {
    if (getAverageAppliedOutput() > 35) {
      containedGamePiece = true;
    } else if (containedGamePiece) {
      lastFiredTimestamp = System.currentTimeMillis();
    }
    SmartDashboard.putNumber("shup RPM", upperRelativeEncoder.getVelocity());
    SmartDashboard.putNumber("shlo RPM", lowerRelativeEncoder.getVelocity());

  }

  /**
   * Gets the timestamp of the latest fire
   *
   * @return empty if never fired, otherwise returns the timestamp in epoch
   *         miliseconds
   */
  public Optional<Long> getLastFiredTimestamp() {
    if (lastFiredTimestamp == -1) {
      return Optional.empty();
    }

    return Optional.of(lastFiredTimestamp);
  }

  /**
   * WARN: may be stale if read too late after game piece was fired.
   *
   * @return whether or not a game piece was fired since this method was last
   *         called. It will return false if a game piece is currently contained
   *         <br>
   */
  public boolean checkGamePieceStatus() {
    if (containedGamePiece && getAverageAppliedOutput() <= 35) {
      containedGamePiece = false;
      return true;
    }

    return false;
  }

  public void run(Modes mode) {
    SmartDashboard.putString(getName() + " mode", mode.toString());

    if (mode == Modes.FORWARD) {
      runWithPower(constants.forwardSpeed);
    } else if (mode == Modes.REVERSE) {
      runWithPower(constants.reverseSpeed);
    } else if (mode == Modes.STOPPED) {
      runWithPower(0);
    }
  }

  public void runWithPower(double power) {
    upper.set(power * ratio.upperPercent());
    lower.set(power * ratio.lowerPercent());
  }

  public void setRatio(MotorRatio ratio) {
    this.ratio = ratio;
  }

  /**
   * @param upperPercent a percentage in [0,1]
   * @param upperPercent a percentage in [0,1]
   */
  public record MotorRatio(double upperPercent, double lowerPercent) {}

  public enum Modes {
    FORWARD,
    REVERSE,
    STOPPED,
  }
}
