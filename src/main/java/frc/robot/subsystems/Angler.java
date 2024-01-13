package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Angler {
    public CANSparkFlex motor;
    public SparkPIDController controller;
    public SparkAbsoluteEncoder absoluteEncoder;
    public Rotation2d setpoint = new Rotation2d();
    
    public Angler (CANSparkFlex motor) {
        this.motor = motor;
        this.controller = motor.getPIDController();
        this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.controller.setP(Constants.IntakeAnglerConstants.p);
        this.controller.setI(Constants.IntakeAnglerConstants.i);
        this.controller.setD(Constants.IntakeAnglerConstants.d);
    }

    public void setRotation (Rotation2d angle) {
        setpoint = angle;
        controller.setReference(MathUtil.clamp(angle.getRadians(), Constants.IntakeAnglerConstants.minAngle, Constants.IntakeAnglerConstants.maxAngle), ControlType.kPosition);
    }

    public void deploy () {
        setRotation(Constants.IntakeAnglerConstants.retractAngle);
    }

    public void retract () {
        setRotation(Constants.IntakeAnglerConstants.detractAngle);
    }

    public boolean isFinished (double tolerance) {
        return Math.abs(setpoint.getRadians() - absoluteEncoder.getPosition()) < tolerance;
    }
}
