package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends Manipulator {

  CANSparkBase duckPrime; // the second duck!
  CANSparkBase kingBob;  // as in the rubber duck not the stupid filet minion

  public Intake(CANSparkBase duckPrime, CANSparkBase kingBob) {
    this.duckPrime = duckPrime;
    this.kingBob = kingBob;
    //kingBob.follow(duckPrime);
    kingBob.setInverted(false);
    kingBob.burnFlash();
     duckPrime.setInverted(false);
    duckPrime.burnFlash();

    this.setName("Intake");
  }

  @Override
  public double getForwardVelocityMultiplier() {
    return Constants.IntakeConstants.outtakePower;
  }

  @Override
  public double getReverseVelocityMultiplier() {
    return Constants.IntakeConstants.intakePower;
  }

  @Override
  public void runNeutral() {
    duckPrime.setIdleMode(IdleMode.kCoast);
    kingBob.setIdleMode(IdleMode.kCoast);
    duckPrime.burnFlash();
    kingBob.burnFlash();
  }

  @Override
  public void stopNeutral() {
    duckPrime.setIdleMode(IdleMode.kBrake);
    kingBob.setIdleMode(IdleMode.kCoast);
    duckPrime.burnFlash();
    kingBob.burnFlash();
  }

  @Override
  public void runWithPower(double power) {
    super.runWithPower(power);
    kingBob.set(power);
    duckPrime.set(power);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Intake currentOut", duckPrime.getOutputCurrent());
      SmartDashboard.putBoolean("Has Gamepice", checkGamePieceStatus());
  }

  private boolean hadGamePiece = false;

  /**
   * @return if the game piece is currently contained within the intake
   */
  @Override
  public boolean checkGamePieceStatus() {
    if(duckPrime.getOutputCurrent() > 30) {
      hadGamePiece = true;
    }else if(duckPrime.get() < 0) {
      hadGamePiece = false;
    }
    
    return hadGamePiece;
  }
}
