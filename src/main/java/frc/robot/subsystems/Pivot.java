package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.RunAnglerCommand.AnglerModes;

public class Pivot extends SubsystemBase {

  public SparkMax motor;
  public SparkMaxConfig motorConfig;
  public SparkClosedLoopController controller;
  public AbsoluteEncoder absoluteEncoder;
  public AbsoluteEncoderConfig absoluteEncoderConfig;
  public Angle setpoint;
  public AnglerModes mode;
  public double massKg;
  protected AnglerConstants constants;
  public boolean isShooter;

  public Pivot(SparkMax motor, AnglerConstants constants, String name) {
    this.motor = motor;
    this.controller = motor.getClosedLoopController();
    this.absoluteEncoder = motor.getAbsoluteEncoder();
    motorConfig.closedLoop
    .p(constants.pid.p())
    .i(constants.pid.i())
    .d(constants.pid.d())
    .outputRange(-.3, constants.speedLimit);
    double convFact = 2 * Math.PI;
    this.setName(name);
    if(constants.enableWrapping) {
      // controller.setPositionPIDWrappingEnabled(true);
      // controller.setPositionPIDWrappingMinInput(0);
      // controller.setPositionPIDWrappingMaxInput(convFact);
    }

    absoluteEncoderConfig.positionConversionFactor(convFact);
    absoluteEncoderConfig.velocityConversionFactor(convFact);
    this.controller = motor.getClosedLoopController();

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.constants = constants;
    this.setpoint = Radian.of(absoluteEncoder.getPosition());

    this.mode = AnglerModes.CUSTOM;
    // calvin commit of the year
  } // 4esahtf v bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm ]]
 
  public Angle getAngleToGround() {
    return setpoint.minus(constants.parallelToGroundAngle);
  }

  public AnglerConstants getAnglerConstants() {
    return this.constants;
  }

  /**
   * @param measure of angle in degrees
   */
  public void setRotation(double measure) {
    setpoint = edu.wpi.first.units.Units.Degree.of(measure);
  }

  public void setRotation(Angle measure) {
    setpoint = measure;
  }

  

  /**
   * Runs to minimum defined angle in AnglerConstants
   */
  public void runToMax() {
    setpoint = IntakeConstants.anglerConstants.mainAngle;
    mode = AnglerModes.STOW;
  }

  /**
   * Runs to minimum defined angle in AnglerConstants
   */
  public void runToMin() {
    setpoint = IntakeConstants.anglerConstants.minAngle;
    mode = AnglerModes.DEPLOY;
  }

  /**
   * @param rate, in degrees
   */
  public void increment(double rate) {
    setpoint = setpoint.plus(edu.wpi.first.units.Units.Degree.of(rate));
    mode = AnglerModes.CUSTOM;
  }

  /**
   * @param tolerance radians
   * @return if current angle is within setpoint tolerance
   */
  public boolean isFinished() {
    return setpoint.isNear(Radians.of(absoluteEncoder.getPosition()), Constants.IntakeConstants.radianTolerance);
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

  @Override
  public void periodic() {

    SmartDashboard.putNumber(getName() + " setpoint deg", setpoint.in(Degrees));
    SmartDashboard.putNumber(getName() + " encoder deg", absoluteEncoder.getPosition() * 180/Math.PI);
    SmartDashboard.putString(getName() + " Mode", this.mode.toString());
    SmartDashboard.putNumber(getName() + " output", motor.getOutputCurrent() / 30);

    double anglerCosMultiplierNoCOMM = constants.weight * 9.81;
    double cosMult = anglerCosMultiplierNoCOMM * constants.lengthMeters;
    double arbFF = cosMult * Math.cos(getAngleToGround().in(Degrees)) / (constants.stalltorque * constants.gearRatio);
    arbFF *= 0.3;
    SmartDashboard.putNumber("Pivot arbFF", arbFF);

    controller.setReference(
        setpoint.in(Radian), 
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        constants.enableFF ? arbFF : 0,
        ArbFFUnits.kPercentOut);
  }
}