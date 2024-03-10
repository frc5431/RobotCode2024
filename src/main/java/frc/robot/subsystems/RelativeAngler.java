package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RelativeAngler extends SubsystemBase {

    private CANSparkFlex right;
    private AbsoluteEncoder rightAbs;
    private RelativeEncoder rightRel;
    private CANSparkFlex left;
    private AbsoluteEncoder leftAbs;
    private RelativeEncoder leftRel;

    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    public double setpoint = 0;

    public RelativeAngler(CANSparkFlex left, CANSparkFlex right) {
        this.left = left;
        this.right = right;

        this.leftAbs = left.getAbsoluteEncoder();
        this.rightAbs = right.getAbsoluteEncoder();
    
        this.leftRel = left.getEncoder();
        this.rightRel = right.getEncoder();

        leftAbs.setPositionConversionFactor(2 * Math.PI);
        rightAbs.setPositionConversionFactor(2 * Math.PI);
    

        leftRel.setPositionConversionFactor(2 * Math.PI);
        rightRel.setPositionConversionFactor(2 * Math.PI);

        leftRel.setPosition(leftAbs.getPosition());
        rightRel.setPosition(rightAbs.getPosition());
        
        left.setInverted(false);
        leftAbs.setInverted(false);
        // leftRel.setInverted(true);
        right.setInverted(true);
        rightAbs.setInverted(true);

        this.leftPID = left.getPIDController();
        this.rightPID = right.getPIDController();

        this.leftPID.setP(0.03);
        this.leftPID.setI(0);
        this.leftPID.setD(0.01);
        
        this.rightPID.setP(0.03);
        this.rightPID.setI(0);
        this.rightPID.setD(0.01);

        leftPID.setFeedbackDevice(leftRel);
        rightPID.setFeedbackDevice(rightRel);
    
        leftPID.setOutputRange(-0.2, 0.2);
        rightPID.setOutputRange(-0.2, 0.2);

        this.left.burnFlash();
        this.right.burnFlash();
    }

    public void setPosition(double rotation) {
        this.setpoint = rotation;
    }   

    public void increament(double rate) {
        this.setpoint += rate;
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public double[] getPositions() {
        return new double[]{leftRel.getPosition(), rightRel.getPosition()};
    }

    @Override
    public void periodic() {
        //how the fuck feedfword?
        leftPID.setReference(
            this.setpoint,
            ControlType.kPosition,
            0
        );

        
        rightPID.setReference(
            setpoint,
            ControlType.kPosition,
            0
        );
    }

    public Command SetAnglerPosition(double setpoint){
        return new StartEndCommand(() -> setPosition(setpoint), () -> {}, this);
    }
    
}
