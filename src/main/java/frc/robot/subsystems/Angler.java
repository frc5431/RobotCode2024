package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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
    
    public Angler (CANSparkFlex motor) {
        this.motor = motor;
        this.controller = motor.getPIDController();
        this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.controller.setP(Constants.AnglerConstants.p);
        this.controller.setI(Constants.AnglerConstants.i);
        this.controller.setD(Constants.AnglerConstants.d);
    }// 4esahtf v       bbbbbbbbbbbbbbbbbbbbbbbbbbb -p[[;lm    ]] 

    public void setRotation (Rotation2d angle) {
        setpoint = angle;
        controller.setReference(MathUtil.clamp(angle.getRadians(), Constants.AnglerConstants.retractAngle.getRadians(), Constants.AnglerConstants.deployAngle.getRadians()), ControlType.kPosition);
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
