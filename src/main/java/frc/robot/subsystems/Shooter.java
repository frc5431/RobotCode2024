package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        this.dtController = distantTop.getPIDController();
        this.dbController = distantBot.getPIDController();

        setGains(mtController);
        setGains(mbController);
        setGains(dtController);
        setGains(dbController);

        mtController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        mbController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        dtController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        dbController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        
        mainTop.setIdleMode(IdleMode.kCoast);
        mainBot.setIdleMode(IdleMode.kCoast);
        distantTop.setIdleMode(IdleMode.kCoast);
        distantBot.setIdleMode(IdleMode.kCoast);

        mtController.setFeedbackDevice(mainTopRel);
        mbController.setFeedbackDevice(mainBotRel);
        dtController.setFeedbackDevice(distantTopRel);
        dbController.setFeedbackDevice(distantBotRel);

        this.mainTop.burnFlash();
        this.mainBot.burnFlash();
        this.distantTop.burnFlash();
        this.distantBot.burnFlash();

    }

    public void setGains(SparkPIDController controller) {
        controller.setP(pid[0]);
        controller.setI(pid[1]);
        controller.setD(pid[2]);
        controller.setIZone(2);
        controller.setOutputRange(-1,1);
    }

    public void RunPair(double percentage, SparkPIDController top, SparkPIDController bot) {
        top.setReference(percentage, ControlType.kDutyCycle);
        bot.setReference(percentage, ControlType.kDutyCycle);
    }

    public void RunPair(double percentage, SparkPIDController top, SparkPIDController bot,  SparkPIDController dtop, SparkPIDController dbot) {
        top.setReference(percentage, ControlType.kDutyCycle);
        bot.setReference(percentage, ControlType.kDutyCycle);
        dtop.setReference(percentage, ControlType.kDutyCycle);
        dbot.setReference(percentage, ControlType.kDutyCycle);
    }

    public void RunPair(double percentage,  double[] ratio, SparkPIDController top, SparkPIDController bot,  SparkPIDController dtop, SparkPIDController dbot) {
        top.setReference(percentage * ratio[0], ControlType.kDutyCycle);
        bot.setReference(percentage * ratio[1], ControlType.kDutyCycle);
        dtop.setReference(percentage * ratio[0], ControlType.kDutyCycle);
        dbot.setReference(percentage * ratio[1], ControlType.kDutyCycle);
    }

    public void RunPair(double percentage, double[] ratio, SparkPIDController top, SparkPIDController bot) {
        top.setReference(percentage * ratio[0], ControlType.kDutyCycle);
        bot.setReference(percentage * ratio[1], ControlType.kDutyCycle);
    }

    public void stopNeutral() {
        RunPair(0, dtController, dbController);
        RunPair(0, mtController, mbController);
        this.mode = ShooterMode.NONE;
    }

    public ShooterMode getMode() {
        return this.mode;  
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Main Set", new Double[]{mainTopRel.getVelocity(), mainBotRel.getVelocity()});
        SmartDashboard.putNumberArray("Distant Set", new Double[]{distantTopRel.getVelocity(), distantBotRel.getVelocity()});
        SmartDashboard.putString("Shooter Mode", this.mode.toString());
    }

    public Command speakerShot() {
        this.mode = ShooterMode.SpeakerShot;
        return new StartEndCommand(() -> RunPair(ShooterConstants.spkSpeed, mtController, mbController), () -> stopNeutral(), this);
    }

    public Command speakerDistantShot() {
        this.mode = ShooterMode.SpeakerDistant;     
        return new StartEndCommand(() -> RunPair(ShooterConstants.stgSpeed, dtController, dbController), () -> stopNeutral(), this);
    }

    public Command stageShot() {
        this.mode = ShooterMode.StageShot;
        return new StartEndCommand(() -> RunPair(ShooterConstants.stgSpeed, mtController, mbController), () -> stopNeutral(), this);
    }

    public Command ampScore() {
        this.mode = ShooterMode.AmpShot;
        return new StartEndCommand(() -> RunPair(ShooterConstants.ampSpeed, ShooterConstants.ampRatio, mtController, mbController),
        () -> stopNeutral(), this);
    } 

    public Command runReverse() {
        return new StartEndCommand(() -> RunPair(ShooterConstants.inSpeed, mtController, mbController, dtController, dtController), () -> stopNeutral(), this);
    }
   
   

}