package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Amper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.leds.Blinkin;
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

  private AnglerConstants intakeAnglerConst;
  private AnglerConstants amperPivotConst;

  private SparkFlex shooterMainTop; 
  private SparkFlex shooterMainBot; 
  private SparkFlex shooterDistantTop;
  private SparkFlex shooterDistantBot;
  private SparkFlex climberLeft;
  private SparkFlex climberRight;

  private SparkMax leftIntakeMotor;
  private SparkMax rightIntakeMotor;
  private SparkMax intakeAnglerMotor;
  private SparkMax amperMotor;
  private SparkMax amperPivotMotor;

  private Blinkin blinkin;

  private MotorType brushless =  MotorType.kBrushless;
  public Drivebase pheonixdrivebase;

  public Systems() {

    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;
    amperPivotConst = Constants.AmperConstants.anglerConstants;

    leftIntakeMotor = new SparkMax(Constants.IntakeConstants.leftIntakeId, brushless);
    rightIntakeMotor = new SparkMax(Constants.IntakeConstants.rightIntakeId, brushless);
    intakeAnglerMotor = new SparkMax(Constants.IntakeConstants.anglerId, brushless);
   
    shooterMainTop = new SparkFlex(Constants.ShooterConstants.mainTopId, brushless);
    shooterMainBot = new SparkFlex(Constants.ShooterConstants.mainBotId, brushless);
    shooterDistantTop = new SparkFlex(Constants.ShooterConstants.distantTopId, brushless);
    shooterDistantBot = new SparkFlex(Constants.ShooterConstants.distantBotId, brushless);
    
    climberLeft = new SparkFlex(Constants.ClimberConstants.leftClimberId, brushless);
    climberRight = new SparkFlex(Constants.ClimberConstants.rightClimberId, brushless);
    amperMotor = new SparkMax(Constants.AmperConstants.amperId, brushless);
    amperPivotMotor = new SparkMax(Constants.AmperConstants.amperPivotId, brushless);
    blinkin = new Blinkin(0);
    
    pheonixdrivebase = new Drivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);
    shooter = new Shooter(shooterMainTop, shooterMainBot, shooterDistantTop, shooterDistantBot);
    intake = new Intake(leftIntakeMotor, rightIntakeMotor, IntakeConstants.manipulatorConstants);
    amper = new Amper(amperMotor);
    amperPivot = new Pivot(amperPivotMotor, amperPivotConst, "amper");
    pivot = new Pivot(intakeAnglerMotor, intakeAnglerConst, "pivot");
    rightClimber = new Climber(climberRight);
    leftClimber = new Climber(climberLeft);

    instance = this;

  }

  public Drivebase getDrivebase() {
    return pheonixdrivebase;
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
