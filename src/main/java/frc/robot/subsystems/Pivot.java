package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.RunAnglerCommand.AnglerModes;

public class Pivot extends SubsystemBase {

  public SparkMax motor;
  public SparkMaxConfig motorConfig = new SparkMaxConfig();
  public SparkClosedLoopController controller;
  public AbsoluteEncoder absoluteEncoder;
  public Angle setpoint;
  public AnglerModes mode;
  public double massKg;
  protected AnglerConstants constants;
  public boolean isShooter;

  public Pivot(SparkMax motor, AnglerConstants constants, String name) {
    this.motor = motor;
    this.controller = motor.getClosedLoopController();
    this.absoluteEncoder = motor.getAbsoluteEncoder();
   
    motorConfig.closedLoop.pid(constants.pid.p(), constants.pid.i(), constants.pid.d());
    motorConfig.closedLoop.outputRange(-0.5,0.5);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    motorConfig.inverted(false);
    motorConfig.absoluteEncoder.inverted(false);
    motorConfig.closedLoop.positionWrappingEnabled(true);
    motorConfig.absoluteEncoder.zeroCentered(false);
    motorConfig.closedLoop.positionWrappingInputRange(0.0, 0.6);
    motorConfig.absoluteEncoder.positionConversionFactor(1);
    this.setName(name);
    motorConfig.idleMode(IdleMode.kBrake);


    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.constants = constants;
    this.setpoint = constants.mainAngle;

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
    setpoint = Units.Rotation.of(measure);
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
   * @param rate, in rotation
   */
  public void increment(double rate) {
    setpoint = setpoint.plus(edu.wpi.first.units.Units.Rotation.of(rate));
    mode = AnglerModes.CUSTOM;
  }

  /**
   * @param tolerance radians
   * @return if current angle is within setpoint tolerance
   */
  public boolean isFinished() {
    return setpoint.isNear(Rotation.of(absoluteEncoder.getPosition()), Constants.IntakeConstants.radianTolerance);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(getName() + " setpoint deg", setpoint.in(Degree));
    SmartDashboard.putNumber(getName() + " encoder deg", Degrees.convertFrom(absoluteEncoder.getPosition(), Rotation));
    SmartDashboard.putString(getName() + " Mode", this.mode.toString());
    SmartDashboard.putNumber(getName() + " output", motor.getAppliedOutput());

    double anglerCosMultiplierNoCOMM = constants.weight * 9.81;
    double cosMult = anglerCosMultiplierNoCOMM * constants.lengthMeters;
    double arbFF = cosMult * Math.cos(getAngleToGround().in(Rotation)) / (constants.stalltorque * constants.gearRatio);
    SmartDashboard.putNumber("Pivot arbFF", arbFF);

    controller.setReference(
        setpoint.in(Rotation), 
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0
        ,
        constants.enableFF ? arbFF : 0,
        ArbFFUnits.kPercentOut
        );
  }
}