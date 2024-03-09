package frc.robot.subsystems;

import java.util.Optional;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnglerConstants;
import frc.robot.commands.RunAnglerCommand.AnglerModes;

public class Angler extends SubsystemBase {

  public CANSparkBase motor;
  public SparkPIDController controller;
  public AbsoluteEncoder absoluteEncoder;
  public Rotation2d setpoint = new Rotation2d();
  public AnglerModes mode;
  public double massKg;
  protected AnglerConstants constants;
  public boolean isShooter;

  public Angler(CANSparkBase motor, AnglerConstants constants, String name) {
    this.motor = motor;
    this.controller = motor.getPIDController();
    this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    this.controller.setP(constants.pid.p());
    this.controller.setI(constants.pid.i());
    this.controller.setD(constants.pid.d());
    isShooter = name.equals("shooter");
   
    controller.setFeedbackDevice(absoluteEncoder);

    // motor.setSmartCurrentLimit(60, 35);
    controller.setOutputRange(-0.6, 0.6);
    double convFact = 2 * Math.PI;
    //controller.setPositionPIDWrappingEnabled(name != "shooter");
    this.setName(name);
      controller.setPositionPIDWrappingEnabled(true);

    
    controller.setPositionPIDWrappingMinInput(0);
    absoluteEncoder.setPositionConversionFactor(convFact);
    absoluteEncoder.setVelocityConversionFactor(convFact);
    controller.setPositionPIDWrappingMaxInput(convFact);

    motor.burnFlash();
    this.constants = constants;
    this.setpoint = Rotation2d.fromRadians(absoluteEncoder.getPosition());

  } // 4esahtf v bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm ]]

  public Rotation2d getAngleToGround() {
    return setpoint.minus(constants.parallelToGroundAngle);
  }

  public void setRotation(Rotation2d angle) {
    setpoint = angle;
  }

  public void runToMax() {
    setRotation(constants.maxAngle);
  }

  public void runToMin() {
    setRotation(constants.minAngle);
  }

  
  /**
   * @param tolerance radians
   * @return
   */
  public boolean isFinished(double tolerance) {
    return Math.abs(setpoint.getRadians() - absoluteEncoder.getPosition()) < tolerance;
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
    SmartDashboard.putNumber(getName() + " setpoint deg", setpoint.getDegrees());
    SmartDashboard.putNumber(getName() + " setpoint radians", setpoint.getRadians());
    SmartDashboard.putNumber(getName() + " encoder deg",
        Units.rotationsToDegrees(absoluteEncoder.getPosition() / absoluteEncoder.getPositionConversionFactor()));
    SmartDashboard.putNumber(getName() + " output", motor.getAppliedOutput());

    // var retractedAngle = constants.minAngle;
    // var deployedAngle = constants.maxAngle;

    double anglerCosMultiplierNoCOMM = massKg * 9.81;
    double cosMult = anglerCosMultiplierNoCOMM * constants.lengthMeters;
    double arbFF = (cosMult * getAngleToGround().getCos()) / constants.stalltorque;
    // SmartDashboard.putNumber(getName() + " arbFF", arbFF);
    // SmartDashboard.putNumber(getName().substring(0, 1) + "2g",
    // getAngleToGround().getDegrees());
    controller.setReference(
        setpoint.getRotations() * absoluteEncoder.getPositionConversionFactor(), // MathUtil.clamp(setpoint.getRadians(),
                                                                                 // retractedAngle, deployedAngle),
        CANSparkBase.ControlType.kPosition,
        0,
        constants.enableFF ? arbFF : 0,
        ArbFFUnits.kPercentOut);
  }
}