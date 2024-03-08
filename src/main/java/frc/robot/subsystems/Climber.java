package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.robot.Constants.ClimberConstants;

public class Climber {
    
  private CANSparkFlex left;
  private CANSparkFlex right;
  private RelativeEncoder relativeEncoder;
  public SparkPIDController controller;
  public double setpoint;

  public Climber (CANSparkFlex left, CANSparkFlex right) {
    this.left = left;
    this.right = right;
    this.controller = right.getPIDController(); 
    this.controller.setP(0.01);
    this.controller.setI(0);
    this.controller.setD(0.1);

    left.setInverted(true);  // dunno if needed
    right.setInverted(true); // dunno if needed
    left.follow(right);
    controller.setFeedbackDevice(relativeEncoder);

    this.left.burnFlash();
    this.right.burnFlash();
  }

  public void setPosition(double height) {
    double downForce = 0.1;

    controller.setReference(
      setpoint,
      CANSparkBase.ControlType.kPosition,
      0,
      ClimberConstants.enableFF ? downForce : 0,
      ArbFFUnits.kPercentOut
    );
  }
}
