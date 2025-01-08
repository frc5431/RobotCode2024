package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.team5431.titan.core.misc.Calc;

public class Shooter extends SubsystemBase {

    private final SparkFlex mainTop;
    private final SparkFlex mainBot;
    private final SparkFlex distantTop;
    private final SparkFlex distantBot;

    private final SparkFlexConfig mainTopConfig;
    private final SparkFlexConfig mainBotConfig;
    private final SparkFlexConfig distantTopConfig;
    private final SparkFlexConfig distantBotConfig;
    
    private final RelativeEncoder mainTopRel;
    private final RelativeEncoder mainBotRel;
    private final RelativeEncoder distantTopRel;
    private final RelativeEncoder distantBotRel;

    private final SparkClosedLoopController mainTopController;
    private final SparkClosedLoopController mainBottomController;
    public final SparkClosedLoopController distantTopController;
    public final SparkClosedLoopController distantBottomController;

    private final ControlType controlType = ControlType.kDutyCycle;

    private double iZone = 0.1;

    private final double[] pid = new double[] { ShooterConstants.p, ShooterConstants.i, ShooterConstants.d };

    public ShooterModes mode;
    public Pair<Double, Double> ratio;

    public Shooter(SparkFlex mainTop, SparkFlex mainBot, SparkFlex distantTop, SparkFlex distantBot) {
        this.mainTop = mainTop;
        this.mainBot = mainBot;
        this.distantTop = distantTop;
        this.distantBot = distantBot;

        this.mainTopRel = mainTop.getEncoder();
        this.mainBotRel = mainBot.getEncoder();
        this.distantTopRel = distantTop.getEncoder();
        this.distantBotRel = distantBot.getEncoder();

        
        this.mainTopController = mainTop.getClosedLoopController();
        this.mainBottomController = mainBot.getClosedLoopController();
        this.distantTopController = distantTop.getClosedLoopController();
        this.distantBottomController = distantBot.getClosedLoopController();

        mainTopConfig.closedLoop
            .p(pid[0])
            .i(pid[1])
            .d(pid[2]);
        
        distantTopConfig.closedLoop
            .p(pid[0])
            .i(pid[1])
            .d(pid[2]);
        
        mainBotConfig.closedLoop
            .p(pid[0])
            .i(pid[1])
            .d(pid[2]);

        distantBotConfig.closedLoop
            .p(pid[0])
            .i(pid[1])
            .d(pid[2]);
        mainTopConfig.idleMode(IdleMode.kCoast);
        mainBotConfig.idleMode(IdleMode.kCoast);
        distantTopConfig.idleMode(IdleMode.kCoast);
        distantBotConfig.idleMode(IdleMode.kCoast);

        // mainTopController.setFeedbackDevice(mainTopRel);
        // mainBottomController.setFeedbackDevice(mainBotRel);
        // distantTopController.setFeedbackDevice(distantTopRel);
        // distantBottomController.setFeedbackDevice(distantBotRel);

        mainTop.configure(mainTopConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        mainBot.configure(mainBotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        distantTop.configure(distantTopConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        distantBot.configure(distantBotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.mode = ShooterModes.NONE;
    }

    public void runPair(double percentage, SparkClosedLoopController top, SparkClosedLoopController bot,
            SparkClosedLoopController dtop, SparkClosedLoopController dbot) {
        top.setReference(percentage * ratio.getFirst(), controlType);
        bot.setReference(percentage * ratio.getSecond(), controlType);
        dtop.setReference(-percentage * ratio.getFirst(), controlType);
        dbot.setReference(-percentage * ratio.getSecond(), controlType);
    }

    public void runPair(double percentage, SparkClosedLoopController top, SparkClosedLoopController bot) {
        top.setReference(percentage * ratio.getFirst(), controlType);
        bot.setReference(percentage * ratio.getSecond(), controlType);
    }

    public void stopNeutral() {
        mode = ShooterModes.NONE;
        runPair(0, distantTopController, distantBottomController);
        runPair(0, mainTopController, mainBottomController);
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

    public void runShooter(ShooterModes mode) {
        this.mode = mode;

        if (mode.ratio.isPresent()) {
            ratio = mode.ratio.get();
        } else {
            ratio = Pair.of(ShooterConstants.shooterRatio.upperPercent(), ShooterConstants.shooterRatio.lowerPercent());
        }

        if (mode.usesMain && mode.usesDistant) {
            runPair(mode.speed, mainTopController, mainBottomController, distantTopController, distantBottomController);
        } else if (mode.usesMain) {
            runPair(mode.speed, mainTopController, mainBottomController);
        } else if (mode.usesDistant) {
            runPair(-mode.speed, distantTopController, distantBottomController);
        }
    }

    public Command runReverse() {
        return new StartEndCommand(
                () -> runPair(ShooterConstants.inSpeed, mainTopController, mainBottomController, distantTopController,
                        distantTopController),
                () -> stopNeutral(), this);
    }

    public Command runShooterCommand(ShooterModes mode) {
        return new StartEndCommand(() -> runShooter(mode), () -> stopNeutral(), this);
    }

    public boolean isClose(double tolerance) {
        double first = tolerance + 1;
        if (mode.usesMain) {
            first = mainTopRel.getVelocity();
        } else if (mode.usesDistant) {
            first = distantTopRel.getVelocity();
        }

        double second = 0;
        if (mode == ShooterModes.SpeakerShot) {
            second = ShooterConstants.mainTopRpm;
        } else if (mode == ShooterModes.SpeakerDistant) {
            second = ShooterConstants.distTopRpm;
        } else if (mode == ShooterModes.DangerDistant) {
            second = ShooterConstants.distDangerRpm;
        } else if (mode == ShooterModes.AmpShot) {
            second = ShooterConstants.mainAmpRpm;
        } else if (mode == ShooterModes.NONE) {
            return false;
        }

        return Calc.approxEquals(Math.abs(first), Math.abs(second), tolerance) && mode.speed != 0;
    }

}