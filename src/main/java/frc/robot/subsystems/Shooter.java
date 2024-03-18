package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterModes;

public class Shooter extends SubsystemBase {

    private final CANSparkFlex mainTop;
    private final CANSparkFlex mainBot;
    private final CANSparkFlex distantTop;
    private final CANSparkFlex distantBot;

    private final RelativeEncoder mainTopRel;
    private final RelativeEncoder mainBotRel;
    private final RelativeEncoder distantTopRel;
    private final RelativeEncoder distantBotRel;

    private final SparkPIDController mainTopController;
    private final SparkPIDController mainBottomController;
    private final SparkPIDController distantTopController;
    private final SparkPIDController distantBottomController;

    private final double[] pid = new double[] { ShooterConstants.p, ShooterConstants.i, ShooterConstants.d };

    public ShooterModes mode;
    public Pair<Double, Double> ratio;

    public Shooter(CANSparkFlex mainTop, CANSparkFlex mainBot, CANSparkFlex distantTop, CANSparkFlex distantBot) {
        this.mainTop = mainTop;
        this.mainBot = mainBot;
        this.distantTop = distantTop;
        this.distantBot = distantBot;

        this.mainTopRel = mainTop.getEncoder();
        this.mainBotRel = mainBot.getEncoder();
        this.distantTopRel = distantTop.getEncoder();
        this.distantBotRel = distantBot.getEncoder();

        this.mainTopController = mainTop.getPIDController();
        this.mainBottomController = mainBot.getPIDController();
        this.distantTopController = distantTop.getPIDController();
        this.distantBottomController = distantBot.getPIDController();

        setGains(mainTopController);
        setGains(mainBottomController);
        setGains(distantTopController);
        setGains(distantBottomController);

        mainTopController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        mainBottomController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        distantTopController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        distantBottomController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        mainTop.setIdleMode(IdleMode.kCoast);
        mainBot.setIdleMode(IdleMode.kCoast);
        distantTop.setIdleMode(IdleMode.kCoast);
        distantBot.setIdleMode(IdleMode.kCoast);

        mainTopController.setFeedbackDevice(mainTopRel);
        mainBottomController.setFeedbackDevice(mainBotRel);
        distantTopController.setFeedbackDevice(distantTopRel);
        distantBottomController.setFeedbackDevice(distantBotRel);

        this.mainTop.burnFlash();
        this.mainBot.burnFlash();
        this.distantTop.burnFlash();
        this.distantBot.burnFlash();

        this.mode = ShooterModes.NONE;
    }

    public void setGains(SparkPIDController controller) {
        controller.setP(pid[0]);
        controller.setI(pid[1]);
        controller.setD(pid[2]);
        controller.setIZone(2);
        controller.setOutputRange(-1, 1);
    }

    public void RunPair(double percentage, SparkPIDController top, SparkPIDController bot,
            SparkPIDController dtop, SparkPIDController dbot) {
        top.setReference(percentage * ratio.getFirst(), ControlType.kDutyCycle);
        bot.setReference(percentage * ratio.getSecond(), ControlType.kDutyCycle);
        dtop.setReference(percentage * ratio.getFirst(), ControlType.kDutyCycle);
        dbot.setReference(percentage * ratio.getSecond(), ControlType.kDutyCycle);
    }

    public void RunPair(double percentage, SparkPIDController top, SparkPIDController bot) {
        top.setReference(percentage * ratio.getFirst(), ControlType.kDutyCycle);
        bot.setReference(percentage * ratio.getSecond(), ControlType.kDutyCycle);
    }

    public void stopNeutral() {
        mode = ShooterModes.NONE;
        RunPair(0, distantTopController, distantBottomController);
        RunPair(0, mainTopController, mainBottomController);
    }

    public ShooterModes getMode() {
        return mode;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Main Set", 
                new Double[] { mainTopRel.getVelocity(), mainBotRel.getVelocity() });
        SmartDashboard.putNumberArray("Distant Set",
                new Double[] { distantTopRel.getVelocity(), distantBotRel.getVelocity() });
        SmartDashboard.putString("Shooter Mode", getMode().toString());

    }

    public Command runReverse() {
        return new StartEndCommand(
                () -> RunPair(ShooterConstants.inSpeed, mainTopController, mainBottomController, distantTopController, distantTopController),
                () -> stopNeutral(), this);
    }

    public void runShooter(ShooterModes mode) {
        this.mode = mode;

        if(mode.ratio.isPresent()) {
            ratio = mode.ratio.get();
        }else {
            ratio = Pair.of(ShooterConstants.shooterRatio.upperPercent(), ShooterConstants.shooterRatio.lowerPercent());
        }

        if(mode.usesMain && mode.usesDistant) {
            RunPair(mode.speed, mainTopController, mainBottomController, distantTopController, distantBottomController);
        } else if(mode.usesMain) {
            RunPair(mode.speed, mainTopController, mainBottomController);
        } else if(mode.usesDistant) {
            RunPair(mode.speed, distantTopController, distantBottomController);
        }else {
            stopNeutral();
        }
    }

    public Command runShooterCommand(ShooterModes mode) {
        return new StartEndCommand(() -> runShooter(mode), () -> stopNeutral(), this);
    }

}