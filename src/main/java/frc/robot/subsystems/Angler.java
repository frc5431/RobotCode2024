package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.RunAnglerCommand.AnglerModes;

public class Angler extends SubsystemBase {
    
    public CANSparkFlex motor;
    public SparkPIDController controller;
    public SparkAbsoluteEncoder absoluteEncoder;
    public Rotation2d setpoint = new Rotation2d();
    public AnglerModes mode;
    public double massKg;
    
    public Angler (CANSparkFlex motor, double massKg) {
        this.motor = motor;
        this.controller = motor.getPIDController();
        this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.controller.setP(Constants.AnglerConstants.p);
        this.controller.setI(Constants.AnglerConstants.i);
        this.controller.setD(Constants.AnglerConstants.d);

        this.massKg = massKg;
    }// 4esahtf v       bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm    ]] 

    public Rotation2d getAngleToGround() {
        return setpoint.minus(Constants.AnglerConstants.parallelToGroundAngle);
    }

    public void setRotation (Rotation2d angle) {
        setpoint = angle;
        var retractedAngle = Constants.AnglerConstants.retractAngle.getRadians();
        var deployedAngle = Constants.AnglerConstants.deployAngle.getRadians();


        double anglerCosMultiplierNoCOMM = massKg * 9.81;
        double cosMult = anglerCosMultiplierNoCOMM * Constants.AnglerConstants.anglerLengthMeters;
        double arbFF = cosMult * getAngleToGround().getCos() / Constants.vortexStallTorque;
        controller.setReference(MathUtil.clamp(angle.getRadians(), retractedAngle, deployedAngle), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
    }

    public void deploy () {
        setRotation(Constants.AnglerConstants.deployAngle);
    }

    public void retract () {
        setRotation(Constants.AnglerConstants.retractAngle);
    }

    public boolean isFinished (double tolerance) {
        return Math.abs(setpoint.getRadians() - absoluteEncoder.getPosition()) < tolerance;
    }
}
