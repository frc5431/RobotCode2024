package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Amper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LasaVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SimpleVision;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.leds.BlinkinPattern;
import frc.robot.subsystems.Drivebase;

public class Systems {
  public static Systems instance;
  private Intake intake;
  private Amper amper;
  private Pivot amperPivot;
  private Pivot pivot;
  private Shooter shooter;
  private Climber rightClimber;
  private Climber leftClimber;

  private SimpleVision simpleVision;

  private AnglerConstants intakeAnglerConst;
  private AnglerConstants amperPivotConst;

  private SparkFlex shooterMainTop; 
  private SparkFlex shooterMainBot; 
  private SparkFlex shooterDistantTop;
  private SparkFlex shooterDistantBot;
  private SparkFlex climberLeft;
  private SparkFlex climberRight;

  private SparkFlex leftIntakeMotor;
  private SparkFlex rightIntakeMotor;
  private SparkFlex intakeAnglerMotor;
  private SparkFlex amperMotor;
  private SparkFlex amperPivotMotor;

  private Blinkin blinkin;

  private MotorType brushless =  MotorType.kBrushless;
  public Drivebase pheonixdrivebase;

  public Systems() {

    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;
    amperPivotConst = Constants.AmperConstants.anglerConstants;

    leftIntakeMotor = new SparkFlex(Constants.IntakeConstants.leftIntakeId, brushless);
    rightIntakeMotor = new SparkFlex(Constants.IntakeConstants.rightIntakeId, brushless);
    intakeAnglerMotor = new SparkFlex(Constants.IntakeConstants.anglerId, brushless);
   
    shooterMainTop = new SparkFlex(Constants.ShooterConstants.mainTopId, brushless);
    shooterMainBot = new SparkFlex(Constants.ShooterConstants.mainBotId, brushless);
    shooterDistantTop = new SparkFlex(Constants.ShooterConstants.distantTopId, brushless);
    shooterDistantBot = new SparkFlex(Constants.ShooterConstants.distantBotId, brushless);
    
    climberLeft = new SparkFlex(Constants.ClimberConstants.leftClimberId, brushless);
    climberRight = new SparkFlex(Constants.ClimberConstants.rightClimberId, brushless);
    amperMotor = new SparkFlex(Constants.AmperConstants.amperId, brushless);
    amperMotor.setInverted(true);
    amperPivotMotor = new SparkFlex(Constants.AmperConstants.amperPivotId, brushless);
    amperPivotMotor.setInverted(true);

    amperMotor.setSmartCurrentLimit(30,25);
    leftIntakeMotor.setSmartCurrentLimit(30,25);
    rightIntakeMotor.setSmartCurrentLimit(30,25);
    rightIntakeMotor.burnFlash();
    leftIntakeMotor.burnFlash();
    intakeAnglerMotor.burnFlash();

    blinkin = new Blinkin(0);
    
    pheonixdrivebase = new Drivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);
    shooter = new Shooter(shooterMainTop, shooterMainBot, shooterDistantTop, shooterDistantBot);
    intake = new Intake(leftIntakeMotor, rightIntakeMotor, IntakeConstants.manipulatorConstants);
    amper = new Amper(amperMotor);
    amperPivot = new Pivot(amperPivotMotor, amperPivotConst, "amper");
    pivot = new Pivot(intakeAnglerMotor, intakeAnglerConst, "pivot");
    rightClimber = new Climber(climberRight);
    leftClimber = new Climber(climberLeft);

    simpleVision = new SimpleVision();
    instance = this;

    // LasaVision.getInstance().setPoseSupplier(() -> pheonixdrivebase.getPose());

  }

  public Drivebase getDrivebase() {
    return pheonixdrivebase;
  }

  public SimpleVision getSimpleVision() {
    return simpleVision;
  }

  public LasaVision getVision() {
    return LasaVision.getInstance();
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Intake getIntake() {
    return intake;
  }

  public Amper getAmper() {
    return amper;
  }

  public Pivot getAmperPivot() {
    return amperPivot;
  }

  public Climber getLeftClimber() {
    return leftClimber;
  }

  public Climber getRightClimber() {
    return rightClimber;
  }

  public Pivot getPivot() {
    return pivot;
  }

  public Blinkin getBlinkin() {
    return blinkin;
  }

}
