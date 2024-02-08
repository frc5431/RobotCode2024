package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends Manipulator {

  CANSparkBase motor;

  public Intake(CANSparkBase motor) {
    this.motor = motor;
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
    motor.burnFlash();
  }

  @Override
  public void stopNeutral() {
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();
  }

  @Override
  public void runWithPower(double power) {
    super.runWithPower(power);
    motor.set(power);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Intake currentOut", motor.getOutputCurrent());
      SmartDashboard.putBoolean("Has Gamepice", checkGamePieceStatus());
  }

  /**
   * @return if the game piece is currently contained within the intake
   */
  @Override
  public boolean checkGamePieceStatus() {
    return motor.getOutputCurrent() > 31;
  }
}
