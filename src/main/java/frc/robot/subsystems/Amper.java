package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmperConstants;
import frc.robot.Constants.AmperConstants.AmperModes;

public class Amper extends SubsystemBase {

  public SparkMax motor;
  public SparkMaxConfig motorConfig = new SparkMaxConfig();
  private DigitalInput beamBreak;

  protected boolean containedGamePiece;

  public RelativeEncoder upperRelativeEncoder;

  private AmperModes mode;

  public Amper(SparkMax motor) {
    this.motor = motor;
    this.mode = AmperModes.STOPPED;
    this.beamBreak = new DigitalInput(8);

    motorConfig.inverted(true);
    motorConfig.smartCurrentLimit(30,25);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    motorConfig.closedLoop.positionWrappingInputRange(-0.1, 0.5);

    this.upperRelativeEncoder = motor.getEncoder();

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    SmartDashboard.putBoolean("amper beambreak", beamBreak.get());

    
  }

  public double[] getRPM() {
    return new double[] { upperRelativeEncoder.getVelocity()};
  }

  public boolean checkGamePieceStatus() {
    return beamBreak.get();
  }

  public void run(AmperModes mode) {
    this.mode = mode;
    if (mode == AmperModes.OUTAKE) {
      runWithPower(AmperConstants.outtakePower);
    } else if (mode == AmperModes.INTAKE) {
      runWithPower(AmperConstants.intakePower);
    } else if (mode == AmperModes.STOPPED) {
      runWithPower(0);
    }
  }

  public void runWithPower(double power) {
    motor.set(0);
  }

  public Command runPower(double power) {
    return new StartEndCommand(() -> runWithPower(power), () -> runWithPower(0), this);
  }

  public Command runMode(AmperModes mode) {
    return new StartEndCommand(() -> run(mode), () -> runWithPower(0), this);
  }

}