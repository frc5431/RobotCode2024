package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
  private CANSparkFlex left;
  private CANSparkFlex right;
  private RelativeEncoder relativeEncoder;

  public SparkPIDController controller;
  public double setpoint;
  public double downForce = 0.1;

  public Climber (CANSparkFlex left, CANSparkFlex right) {
    this.left = left;
    this.right = right;
    this.controller = right.getPIDController(); 
    this.controller.setP(0.5);
    this.controller.setI(0);
    this.controller.setD(0.1);
    this.controller.setOutputRange(-0.2, 0.2);

    relativeEncoder = right.getEncoder();

    left.follow(right, true);
    this.left.burnFlash();
    this.right.burnFlash();
    controller.setFeedbackDevice(relativeEncoder);
  }

  public void rawDrive(double percentage) {
    right.set(percentage);
  }

  public void setPosition(double angle) {
    this.setpoint = angle;
  }

  public double getposition() {
    return setpoint;
  }

  @Override public void periodic() {
    SmartDashboard.putNumber("climber setpoint", setpoint);
    SmartDashboard.putNumber("climber encoder", relativeEncoder.getPosition());
   controller.setReference(
      setpoint,
      CANSparkBase.ControlType.kPosition,
      0
    );
  
  }

   public Command SetAnglerPosition(double setpoint){
        return new StartEndCommand(() -> setPosition(setpoint), () -> {}, this);
    }


}
