package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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

    public double setpoint;

    public RelativeAngler(CANSparkFlex left, CANSparkFlex right) {
        this.right = right;
        this.left = left;

        this.rightAbs = left.getAbsoluteEncoder();
        this.leftAbs = left.getAbsoluteEncoder();

        leftAbs.setPositionConversionFactor(2 * Math.PI);
        rightAbs.setPositionConversionFactor(2 * Math.PI);
        
        this.leftRel = left.getEncoder();
        this.rightRel = left.getEncoder();

        leftRel.setPositionConversionFactor(2 * Math.PI);
        rightRel.setPositionConversionFactor(2 * Math.PI);

        leftRel.setPosition(leftAbs.getPosition());
        rightRel.setPosition(rightAbs.getPosition());
        
        left.setInverted(true);
        leftRel.setInverted(true);

        this.leftPID.setP(0.2);
        this.leftPID.setI(0);
        this.leftPID.setD(0.01);
        
        this.rightPID.setP(0.2);
        this.rightPID.setI(0);
        this.rightPID.setD(0.01);

        leftPID.setFeedbackDevice(leftRel);
        rightPID.setFeedbackDevice(rightRel);
    
        leftPID.setOutputRange(-0.8, 0.8);
        rightPID.setOutputRange(-0.8, 0.8);

        this.left.burnFlash();
        this.right.burnFlash();
    }

    public void setPosition(double rotation) {
        this.setpoint = rotation;
    }   

    public void increament(double rate) {
        this.setpoint += rate;
    }


    @Override
    public void periodic() {
        //how the fuck feedfword?
        leftPID.setReference(
            setpoint,
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
        return new StartEndCommand(() -> setPosition(setpoint), null, this);
    }
    



}
