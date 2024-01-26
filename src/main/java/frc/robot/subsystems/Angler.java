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
import frc.robot.Constants.AnglerConstants;
import frc.robot.commands.RunAnglerCommand.AnglerModes;

public class Angler extends SubsystemBase {
    
    public CANSparkFlex motor;
    public SparkPIDController controller;
    public SparkAbsoluteEncoder absoluteEncoder;
    public Rotation2d setpoint = new Rotation2d();
    public AnglerModes mode;
    public double massKg;
    protected AnglerConstants constants;
    
    public Angler (CANSparkFlex motor, AnglerConstants constants, String name) {
        this.motor = motor;
        this.controller = motor.getPIDController();
        this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.controller.setP(constants.pid.p());
        this.controller.setI(constants.pid.i());
        this.controller.setD(constants.pid.d());

        this.constants = constants;
        this.setName(name);
    }// 4esahtf v       bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm    ]] 

    public Rotation2d getAngleToGround() {
        return setpoint.minus(constants.parallelToGroundAngle);
    }

    public void setRotation(Rotation2d angle) {
        setpoint = angle;
        var retractedAngle = constants.minAngle.getRadians();
        var deployedAngle = constants.maxAngle.getRadians();


        double anglerCosMultiplierNoCOMM = massKg * 9.81;
        double cosMult = anglerCosMultiplierNoCOMM * constants.lengthMeters;
        double arbFF = cosMult * getAngleToGround().getCos() / Constants.vortexStallTorque;
        controller.setReference(MathUtil.clamp(angle.getRadians(), retractedAngle, deployedAngle), ControlType.kPosition, 0, arbFF, ArbFFUnits.kPercentOut);
    }

    public void deploy() {
        setRotation(constants.maxAngle);
    }

    public void retract() {
        setRotation(constants.minAngle);
    }

    public boolean isFinished(double tolerance) {
        return Math.abs(setpoint.getRadians() - absoluteEncoder.getPosition()) < tolerance;
    }
}
