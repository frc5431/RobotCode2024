package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnglerConstants;

public class Angler extends SubsystemBase {

  public CANSparkBase motor;
  public SparkPIDController controller;
  public AbsoluteEncoder absoluteEncoder;
 // public Rotation2d setpoint = new Rotation2d(); 
  public Measure<Angle> setpoint;
  public double massKg;
  protected AnglerConstants constants;
  public boolean isShooter;

  public Angler(CANSparkBase motor, AnglerConstants constants, String name) {
    this.motor = motor;
    this.controller = motor.getPIDController();
    this.absoluteEncoder = motor.getAbsoluteEncoder();
    this.controller.setP(constants.pid.p());
    this.controller.setI(constants.pid.i());
    this.controller.setD(constants.pid.d());


    // motor.setSmartCurrentLimit(60, 35);
    controller.setOutputRange(-0.8, 0.8);
    double convFact = 2 * Math.PI;
    this.setName(name);
    controller.setPositionPIDWrappingEnabled(true);

    controller.setPositionPIDWrappingMinInput(0);
    absoluteEncoder.setPositionConversionFactor(convFact);
    absoluteEncoder.setVelocityConversionFactor(convFact);
    controller.setPositionPIDWrappingMaxInput(convFact);
    controller.setFeedbackDevice(absoluteEncoder);
    motor.setSoftLimit(SoftLimitDirection.kForward, ((float)constants.maxAngle.in(Units.Degrees)));
    motor.setSoftLimit(SoftLimitDirection.kForward, ((float)constants.minAngle.in(Units.Degrees)));


    motor.burnFlash();
    this.constants = constants;
    this.setpoint = Units.Radians.of(absoluteEncoder.getPosition());
    // calvin commit of the year
  } // 4esahtf v bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm ]]
 
  public Measure<Angle> getAngleToGround() {
    return setpoint.minus(Units.Radian.of(constants.parallelToGroundAngle));
  }

  public void setRotation(double measure) {
    setpoint = Units.Degree.of(measure);
  }
  
  public void increment(double rate) {
    setpoint.plus(Units.Degree.of(rate));
  }

  /**
   * @param tolerance radians
   * @return
   */
  public boolean isFinished() {
    return Math.abs(setpoint.minus(Units.Radians.of(absoluteEncoder.getPosition())).magnitude()) < 0.3;
  }

  public Command runToAngleInstant(Measure<Angle> rotation) {
    return new RunCommand(() -> setpoint = rotation, this);
  }

  public Command runToAngleUntilFinished(Measure<Angle> rotation) {
    return new RunCommand(() -> setpoint = rotation, this).until(this::isFinished);
  }

  public Command runToMinimum() {
    return runToAngleInstant(constants.minAngle);
  }

  public Command runToMaximum() {
    return runToAngleInstant(constants.maxAngle);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(getName() + " setpoint deg", setpoint.magnitude());
    SmartDashboard.putNumber(getName() + " encoder deg", absoluteEncoder.getPosition());

    double anglerCosMultiplierNoCOMM = massKg * 9.81;
    double cosMult = anglerCosMultiplierNoCOMM * constants.lengthMeters;
    double arbFF = (cosMult * 1) / constants.stalltorque;

    controller.setReference(
        setpoint.magnitude() *  absoluteEncoder.getPositionConversionFactor(), // MathUtil.clamp(setpoint.getRadians(),
                                                                                 // retractedAngle, deployedAngle),
        CANSparkBase.ControlType.kPosition,
        0,
        constants.enableFF ? arbFF : 0,
        ArbFFUnits.kPercentOut);
  }

  public AnglerConstants getConstants() {
    return constants;
  }

    /*
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
      Translation2d velocityVector = new Translation2d(velocity * Math.cos(angleInRadians),
          velocity * Math.sin(angleInRadians));

      double timeOfFlight = velocityVector.getY() / 9.81;
      double calculatedDistanceX = velocityVector.getX() * timeOfFlight;
      double calculatedHeightY = velocityVector.getY() / 2 * gravitationalConstant;

      Angle angle = new Angle(Rotation2d.fromRadians(angleInRadians),
          new Translation2d(Math.abs(calculatedDistanceX - distanceX), Math.abs(calculatedHeightY - heightY)));
      if (bestAngle.isEmpty()
          || bestAngle.get().getSimpleError() < angle.getSimpleError() && angle.getSimpleError() < 1) {
        bestAngle = Optional.of(angle);
      }
    }

    return Optional.empty(); // No suitable angle found
  }
  */
}