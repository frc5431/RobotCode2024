package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.WeightedAverageController;

public class TitanFieldCentricFacingAngle implements SwerveRequest {

    public double velocityX;
    public double velocityY;
    public double targetHeading = 0;
    public PIDController pid;
    public Pigeon2 gyro;
    public WeightedAverageController headingDampening;
    // the classic WPI_GAIMFBERLMLMNFTCCZZEController

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        double dampenedValue = headingDampening.calculate(targetHeading);
        double rotationRate = pid.calculate(edu.wpi.first.math.util.Units.degreesToRadians(gyro.getAngle() % 360), dampenedValue);

        SmartDashboard.putNumber("gyroRads", edu.wpi.first.math.util.Units.degreesToRadians(gyro.getAngle()));
        SmartDashboard.putNumber("dampenedValue", dampenedValue);

        double toApplyOmega = rotationRate;

        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(velocityX, velocityY, toApplyOmega,
                parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        }

        return StatusCode.OK;
    }

    public TitanFieldCentricFacingAngle withHeading(double targetHeading) {
        this.targetHeading = targetHeading;
        return this;
    }

    public TitanFieldCentricFacingAngle withVelocityX(double velocityX) {
        this.velocityX = velocityX;
        return this;
    }

    public TitanFieldCentricFacingAngle withVelocityY(double velocityY) {
        this.velocityY = velocityY;
        return this;
    }

    public TitanFieldCentricFacingAngle withPID(PIDController pid) {
        this.pid = pid;
        this.pid.enableContinuousInput(0, 2 * Math.PI);
        return this;
    }

    public TitanFieldCentricFacingAngle withDampening(WeightedAverageController dampener) {
        this.headingDampening = dampener;
        return this;
    }
    
}