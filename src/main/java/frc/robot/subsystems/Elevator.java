package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private CANSparkFlex leftMotor; // baby motor
    private CANSparkFlex rightMotor; //papa motor
    // private SparkRelativeEncoder leftMotorEncoder;
    private SparkRelativeEncoder rightMotorEncoder;
    private SparkPIDController controller;
    // private Rotation2d leftSetpoint;
    private Rotation2d rightSetpoint = new Rotation2d();
    protected ElevatorConstants constants;
    public double massKg;
    

    public Elevator(ElevatorConstants constants, CANSparkFlex leftMotor, CANSparkFlex rightMotor){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.controller = rightMotor.getPIDController();
        this.constants = constants;
        // leftSetpoint = Rotation2d.fromRadians(leftMotorEncoder.getPosition());
        this.rightSetpoint = Rotation2d.fromRadians(rightMotorEncoder.getPosition());
        this.controller.setP(constants.pid.p());
        this.controller.setI(constants.pid.i());
        this.controller.setD(constants.pid.d());

        controller.setFeedbackDevice(rightMotorEncoder);

        rightMotor.setInverted(false);
        leftMotor.setInverted(false);
        leftMotor.follow(rightMotor);
        this.leftMotor.burnFlash();
        this.rightMotor.burnFlash();
    }

    public void setPosition(Rotation2d rotations){
        rightSetpoint = rotations;
    }

    public void setHigh(){
        setPosition(constants.maxHeight);
    }
    
    public void setLow(){
        setPosition(constants.minHeight);
    }

    public Rotation2d getLengthToGround() {
        return rightSetpoint.minus(constants.maxHeight);
    }

    public void periodic(){
        double anglerCosMultiplierNoCOMM = massKg * 9.81;
        double cosMult = anglerCosMultiplierNoCOMM * constants.lengthMeters;
        double arbFF = (cosMult * getLengthToGround().getCos()) / constants.stalltorque;

        controller.setReference(
        rightSetpoint.getRotations() * rightMotorEncoder.getPositionConversionFactor(), //MathUtil.clamp(setpoint.getRadians(), retractedAngle, deployedAngle),
        CANSparkBase.ControlType.kPosition,
        0,
        constants.enableFF ? arbFF : 0,
        ArbFFUnits.kPercentOut
    );
    }

}
