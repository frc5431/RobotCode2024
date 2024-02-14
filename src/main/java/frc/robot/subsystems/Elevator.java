package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private CANSparkFlex left;
  private CANSparkFlex right;
  private RelativeEncoder relativeEncoder;
  public SparkPIDController controller;

  public Elevator (CANSparkFlex left, CANSparkFlex right) {
    this.left = left;
    this.right = right;
    this.controller = right.getPIDController(); 
    this.controller.setP(ElevatorConstants.pid.p());
    this.controller.setI(ElevatorConstants.pid.i());
    this.controller.setD(ElevatorConstants.pid.d());

    left.setInverted(true);  // dunno if needed
    right.setInverted(true); // dunno if needed
    left.follow(right);
    controller.setFeedbackDevice(relativeEncoder);

    this.left.burnFlash();
    this.right.burnFlash();
  }

  public void setPosition () {
    double downForce = Units.lbsToKilograms(ElevatorConstants.roboWeight) * 9.81;

    controller.setReference(
      ElevatorConstants.maxHeight,
      CANSparkBase.ControlType.kPosition,
      0,
      ElevatorConstants.enableFF ? downForce : 0,
      ArbFFUnits.kPercentOut
    );
  }
}
