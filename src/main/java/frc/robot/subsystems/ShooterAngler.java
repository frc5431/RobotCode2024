package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnglerConstants;
import frc.robot.commands.RunAnglerCommand.AnglerModes;

public class ShooterAngler extends SubsystemBase {
    
  public CANSparkBase motor;
  public SparkPIDController controller;
  public AbsoluteEncoder absoluteEncoder;
  public double setpoint = 0;
  public AnglerModes mode;
  public double massKg;
  protected AnglerConstants constants;

  public ShooterAngler(CANSparkBase motor, AnglerConstants constants, String name) {
    this.motor = motor;
    this.controller = motor.getPIDController();
    this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    this.controller.setP(constants.pid.p());
    this.controller.setI(constants.pid.i());
    this.controller.setD(constants.pid.d());

    controller.setFeedbackDevice(absoluteEncoder);
    controller.setOutputRange(-constants.speedLimit, constants.speedLimit);
    // double convFact = 2 * Math.PI;
    this.setName(name);
    
    // absoluteEncoder.setPositionConversionFactor(convFact);
    // absoluteEncoder.setVelocityConversionFactor(convFact);

    motor.burnFlash();
    this.constants = constants;
    this.setpoint = absoluteEncoder.getPosition();

  } // 4esahtf v bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm ]]

  public void setRotation(double angle) {
    setpoint = angle;
  }

  public void runToMax() {
    setRotation(3 * Math.PI);
  }

  public void runToMin() {
    setRotation(0);
  }


  record Angle(Rotation2d angle, Translation2d error) {
    double getSimpleError() {
      return Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY());
    }
  }

  public static Optional<Double> calculateLaunchAngle(double velocity, double distanceX, double heightY) {
    // Iteratively searching for a launch angle that allows hitting the target
    Optional<Angle> bestAngle = Optional.empty();
    for (int angleInDegrees = 0; angleInDegrees <= 90; angleInDegrees += 1) {
      final double gravitationalConstant = 9.81;
      double angleInRadians = Units.degreesToRadians(angleInDegrees);
      Translation2d velocityVector = new Translation2d(velocity * Math.cos(angleInRadians), velocity * Math.sin(angleInRadians));

      double timeOfFlight = velocityVector.getY() / 9.81;
      double calculatedDistanceX = velocityVector.getX() * timeOfFlight;
      double calculatedHeightY = velocityVector.getY() / 2 * gravitationalConstant;

      Angle angle = new Angle(Rotation2d.fromRadians(angleInRadians), new Translation2d(Math.abs(calculatedDistanceX - distanceX), Math.abs(calculatedHeightY - heightY)));
      if(bestAngle.isEmpty() || bestAngle.get().getSimpleError() < angle.getSimpleError() && angle.getSimpleError() < 1) {
        bestAngle = Optional.of(angle);
      }
    }


    return Optional.empty(); // No suitable angle found
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + " setpoint deg", Units.radiansToDegrees(setpoint));
    SmartDashboard.putNumber(getName() + " setpoint radians", setpoint);
    SmartDashboard.putNumber(getName() + " encoder deg",
        Units.rotationsToDegrees(absoluteEncoder.getPosition() / absoluteEncoder.getPositionConversionFactor()));
    SmartDashboard.putNumber(getName() + " output", motor.getAppliedOutput());
    controller.setReference(setpoint,CANSparkBase.ControlType.kPosition);
  }

  public boolean isFinished(double tolerance) {
    return Math.abs(setpoint - absoluteEncoder.getPosition()) < tolerance;
  }
}