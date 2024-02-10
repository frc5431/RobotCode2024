package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends Manipulator {

  CANSparkBase motor;
  CANSparkBase kingBob; // as in the rubber duck not the stupid filet minion

  public Intake(CANSparkBase motor, CANSparkBase kingBob) {
    this.motor = motor;
    this.kingBob = kingBob;
    //kingBob.follow(motor);
    kingBob.setInverted(false);
    kingBob.burnFlash();
     motor.setInverted(false);
    motor.burnFlash();

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
    motor.setIdleMode(IdleMode.kCoast);
    kingBob.setIdleMode(IdleMode.kCoast);
    motor.burnFlash();
    kingBob.burnFlash();
  }

  @Override
  public void stopNeutral() {
    motor.setIdleMode(IdleMode.kBrake);
    kingBob.setIdleMode(IdleMode.kCoast);
    motor.burnFlash();
    kingBob.burnFlash();
  }

  @Override
  public void runWithPower(double power) {
    super.runWithPower(power);
    kingBob.set(power);
    motor.set(power);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Intake currentOut", motor.getOutputCurrent());
      SmartDashboard.putBoolean("Has Gamepice", checkGamePieceStatus());
  }

  private boolean hadGamePiece = false;

  /**
   * @return if the game piece is currently contained within the intake
   */
  @Override
  public boolean checkGamePieceStatus() {
    if(motor.getOutputCurrent() > 30) {
      hadGamePiece = true;
    }else if(motor.get() < 0) {
      hadGamePiece = false;
    }
    
    return hadGamePiece;
  }
}
