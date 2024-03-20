package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LasaVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivebase;

public class Systems {
  public static Systems instance;
  private Intake intake;
  private Pivot pivot;
  private Shooter shooter;
  private Climber climber;

  private AnglerConstants intakeAnglerConst;

  private CANSparkFlex shooterMainTop; 
  private CANSparkFlex shooterMainBot; 
  private CANSparkFlex shooterDistantTop;
  private CANSparkFlex shooterDistantBot;
  private CANSparkFlex climberLeft;
  private CANSparkFlex climberRight;

  private CANSparkMax leftIntakeMotor;
  private CANSparkMax rightIntakeMotor;
  private CANSparkMax intakeAnglerMotor;

  private MotorType brushless =  MotorType.kBrushless;
  public Drivebase pheonixdrivebase;

  public Systems() {

    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;

    leftIntakeMotor = new CANSparkMax(Constants.IntakeConstants.leftIntakeId, brushless);
    rightIntakeMotor = new CANSparkMax(Constants.IntakeConstants.rightIntakeId, brushless);
    intakeAnglerMotor = new CANSparkMax(Constants.IntakeConstants.anglerId, brushless);
   
    shooterMainTop = new CANSparkFlex(Constants.ShooterConstants.mainTopId, brushless);
    shooterMainBot = new CANSparkFlex(Constants.ShooterConstants.mainBotId, brushless);
    shooterDistantTop = new CANSparkFlex(Constants.ShooterConstants.distantTopId, brushless);
    shooterDistantBot = new CANSparkFlex(Constants.ShooterConstants.distantBotId, brushless);
    
    climberLeft = new CANSparkFlex(Constants.ClimberConstants.leftClimberId, brushless);
    climberRight = new CANSparkFlex(Constants.ClimberConstants.rightClimberId, brushless);
  
    leftIntakeMotor.setSmartCurrentLimit(30,25);
    rightIntakeMotor.setSmartCurrentLimit(30,25);
    rightIntakeMotor.burnFlash();
    leftIntakeMotor.burnFlash();
    intakeAnglerMotor.burnFlash();
    
    pheonixdrivebase = new Drivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);
    shooter = new Shooter(shooterMainTop, shooterMainBot, shooterDistantTop, shooterDistantBot);
    intake = new Intake(leftIntakeMotor, rightIntakeMotor, IntakeConstants.manipulatorConstants);
    pivot = new Pivot(intakeAnglerMotor, intakeAnglerConst, "pivot");
    climber = new Climber(climberLeft, climberRight);
    instance = this;

    // LasaVision.getInstance().setPoseSupplier(() -> pheonixdrivebase.getPose());

  }

  public Drivebase getDrivebase() {
    return pheonixdrivebase;
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

  public Climber getClimber() {
    return climber;
  }

  public Pivot getPivot() {
    return pivot;
  }

}
