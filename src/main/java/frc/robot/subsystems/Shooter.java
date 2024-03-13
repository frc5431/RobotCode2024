package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterMode;

public class Shooter extends SubsystemBase {
    
    private final CANSparkFlex mainTop;
    private final CANSparkFlex mainBot;
    private final CANSparkFlex distantTop;
    private final CANSparkFlex distantBot;
    
    private final RelativeEncoder mainTopRel;
    private final RelativeEncoder mainBotRel;
    private final RelativeEncoder distantTopRel;
    private final RelativeEncoder distantBotRel;

    private final SparkPIDController mtController;
    private final SparkPIDController mbController;
    private final SparkPIDController dtController;
    private final SparkPIDController dbController;

    private final double[] pid = new double[] {ShooterConstants.p, ShooterConstants.i, ShooterConstants.d};

    public ShooterMode mode;

    public Shooter(CANSparkFlex mainTop, CANSparkFlex mainBot, CANSparkFlex distantTop, CANSparkFlex distantBot) {
        this.mainTop = mainTop;
        this.mainBot = mainBot;
        this.distantTop = distantTop;
        this.distantBot = distantBot;

        this.mainTopRel = mainTop.getEncoder();
        this.mainBotRel = mainBot.getEncoder();
        this.distantTopRel = distantTop.getEncoder();
        this.distantBotRel = distantBot.getEncoder();

        this.mtController = mainTop.getPIDController();
        this.mbController = mainBot.getPIDController();
        this.dtController = mainTop.getPIDController();
        this.dbController = mainBot.getPIDController();

        setGains(mtController);
        setGains(mbController);
        setGains(dtController);
        setGains(dbController);

        mtController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        mbController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        dtController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        dbController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        mtController.setFeedbackDevice(mainTopRel);
        mbController.setFeedbackDevice(mainBotRel);
        dtController.setFeedbackDevice(distantTopRel);
        dbController.setFeedbackDevice(distantBotRel);
        
        mainTop.setIdleMode(IdleMode.kCoast);
        mainBot.setIdleMode(IdleMode.kCoast);
        distantTop.setIdleMode(IdleMode.kCoast);
        distantTop.setIdleMode(IdleMode.kCoast);

    }

    public void setGains(SparkPIDController controller) {
        controller.setP(pid[0]);
        controller.setI(pid[1]);
        controller.setD(pid[2]);
        controller.setIZone(2);
    }

    public void RunPair(double percentage, SparkPIDController top, SparkPIDController bot) {
        top.setReference(percentage, ControlType.kVelocity);
        bot.setReference(percentage, ControlType.kVelocity);
    }

    public void spkShot() {
        this.mode = ShooterMode.SpeakerShot;
        RunPair(ShooterConstants.spkSpeed, mtController, mbController);
    }

    public void stgShot() {
        this.mode = ShooterMode.StageShot;
        RunPair(ShooterConstants.spkSpeed, mtController, mbController);
    }

    public void ampShot() {
        this.mode = ShooterMode.AmpShot;
        RunPair(ShooterConstants.ampSpeed, mtController, mbController);
    }

    public void mainIn() {
        this.mode = ShooterMode.MainIn;
        RunPair(ShooterConstants.inSpeed, mtController, mbController);
    }

    public void spkDistantShot() {
        this.mode = ShooterMode.SpeakerDistant;
        RunPair(ShooterConstants.stgSpeed, dtController, dbController);
    }
    
    public void distantIn() {
        this.mode = ShooterMode.MainIn;
        RunPair(ShooterConstants.inSpeed, dtController, dbController);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Main Set", new Double[]{mainTopRel.getVelocity(), mainBotRel.getVelocity()});
        SmartDashboard.putNumberArray("Distant Set", new Double[]{distantTopRel.getVelocity(), distantBotRel.getVelocity()});

    }

    public Command spkScore() {
        return new StartEndCommand(() -> spkShot(), () -> {}, this);
    }

    public Command spkDistant() {
        return new StartEndCommand(() -> spkDistantShot(), () -> {}, this);
    }

    public Command stgScore() {
        return new StartEndCommand(() -> stgShot(), () -> {}, this);
    }

    public Command ampScore() {
        return new StartEndCommand(() -> ampShot(), () -> {}, this);
    } 

    public Command mainReverse() {
        return new StartEndCommand(() -> mainIn(), () -> {}, this);
    } 

    public Command distantReverse() {
        return new StartEndCommand(() -> distantIn(), () -> {}, this);
    } 



}