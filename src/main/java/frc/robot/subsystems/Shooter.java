package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.Optional;

public class Shooter extends Manipulator {

  public CANSparkFlex upper;
  public CANSparkFlex lower;

  protected boolean containedGamePiece;
  private long lastFiredTimestamp = -1;
  protected ShooterRatio ratio;

  public RelativeEncoder upperRelativeEncoder;
  public RelativeEncoder lowerRelativeEncoder;

  public Shooter(CANSparkFlex upper, CANSparkFlex lower) {
    this.setName("Shooter");
    this.upper = upper;
    this.lower = lower;

    upper.setInverted(true);
    lower.setInverted(true);
    this.upper.burnFlash();
    this.lower.burnFlash();
  }

  @Override
  public double getForwardVelocityMultiplier() {
    return Constants.ShooterConstants.normalPower;
  }

  // not sure why we'd ever reverse, but here it is i guess...
  @Override
  public double getReverseVelocityMultiplier() {
    return getForwardVelocityMultiplier() / 4;
  }

  @Override
  public void runNeutral() {
    this.upper.setIdleMode(IdleMode.kCoast);
    this.upper.burnFlash();
    this.lower.setIdleMode(IdleMode.kCoast);
    this.lower.burnFlash();
  }

  @Override
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
  @Override
  public boolean checkGamePieceStatus() {
    if (containedGamePiece && getAverageAppliedOutput() <= 35) {
      containedGamePiece = false;
      return true;
    }

    return false;
  }

  @Override
  public void runWithPower(double power) {
    upper.set(power * ratio.upperPercent());
    lower.set(power * ratio.lowerPercent());
  }

  public void setRatio(ShooterRatio ratio) {
    this.ratio = ratio;
  }

  /**
   * @param upperPercent a percentage in [0,1]
   * @param upperPercent a percentage in [0,1]
   */
  public record ShooterRatio(double upperPercent, double lowerPercent) {}
}
