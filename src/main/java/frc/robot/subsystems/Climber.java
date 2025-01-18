package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
  public SparkFlex climbMotor;
  private SparkFlexConfig rightConfig = new SparkFlexConfig();
  public SparkClosedLoopController controller;
  public double setpoint;
  public double downForce = 0.1;
  public ClimberModes mode;

  public enum ClimberModes {
    UP,
    DOWN,
    STOPPED
  }

  public Climber (SparkFlex climbMotor) {
    this.climbMotor = climbMotor;
    rightConfig.closedLoop
      .p(0.3)
      .i(0)
      .d(.1)
      .outputRange(-1, 1);
    this.controller = climbMotor.getClosedLoopController();
    this.mode = ClimberModes.STOPPED;
    climbMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    SmartDashboard.putNumber("Climber Encoder", climbMotor.getEncoder().getPosition());
    SmartDashboard.putString("Climber Mode", mode.toString());
    controller.setReference(
      setpoint,
      SparkBase.ControlType.kPosition,
      ClosedLoopSlot.kSlot0
    );

  }

  public Command increment(double rate) {
    return new InstantCommand(() -> incrementPosition(rate), this);
  }

}
