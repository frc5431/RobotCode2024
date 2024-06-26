package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
  private CANSparkFlex right;
  public RelativeEncoder relativeEncoder;
  public SparkPIDController controller;
  public double setpoint;
  public double downForce = 0.1;
  public ClimberModes mode;

  public enum ClimberModes {
    UP,
    DOWN,
    STOPPED
  }

  public Climber (CANSparkFlex climbMotor) {
    this.right = climbMotor;
    this.controller = climbMotor.getPIDController(); 
    this.controller.setP(0.3);
    this.controller.setI(0);
    this.controller.setD(0.1);
    this.controller.setOutputRange(-1, 1);
    this.relativeEncoder = climbMotor.getEncoder();
    controller.setFeedbackDevice(relativeEncoder);
    this.mode = ClimberModes.STOPPED;
    this.right.burnFlash();
  }

  public void setPosition(double angle) {
    this.setpoint = angle;
  }

  public void incrementPosition(double rate) {
    this.setpoint += rate;
  }

  public double getposition() {
    return setpoint;
  }

  @Override public void periodic() {
    SmartDashboard.putNumber("Climber Setpoint", setpoint);
    SmartDashboard.putNumber("Climber Encoder", relativeEncoder.getPosition());
    SmartDashboard.putString("Climber Mode", mode.toString());
    controller.setReference(
      setpoint,
      CANSparkBase.ControlType.kPosition,
      0
    );

  }

  public Command increment(double rate) {
    return new InstantCommand(() -> incrementPosition(rate), this);
  }

}
