package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LasaVision;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PheonixDrivebase;

public class Systems {
  public static Systems instance;
  // private Vision vision;
  private Manipulator intake;
  private Angler pivot;
  private Angler shooterAngler;
  private Manipulator shooter;
  private Climber climber;
  private DigitalInput beambreak;

  private AnglerConstants intakeAnglerConst;
  private AnglerConstants shooterAnglerConst;


  private CANSparkFlex shooterUpper; 
  private CANSparkFlex shooterLower; 
  private CANSparkFlex anglerLeft;
  private CANSparkFlex anglerRight;
  private CANSparkFlex climberLeft;
  private CANSparkFlex climberRight;

  private CANSparkMax leftIntakeMotor;
  private CANSparkMax rightIntakeMotor;
  private CANSparkMax intakeAnglerMotor;


  private MotorType brushless =  MotorType.kBrushless;
  public PheonixDrivebase pheonixdrivebase;

  public Systems() {



    shooterAnglerConst = Constants.ShooterConstants.anglerConstants;
    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;


    leftIntakeMotor = new CANSparkMax(Constants.IntakeConstants.leftIntakeId, brushless);
    rightIntakeMotor = new CANSparkMax(Constants.IntakeConstants.rightIntakeId, brushless);
    intakeAnglerMotor = new CANSparkMax(Constants.IntakeConstants.anglerId, brushless);


    shooterLower = new CANSparkFlex(Constants.ShooterConstants.botId, brushless);
    shooterUpper = new CANSparkFlex(Constants.ShooterConstants.topId, brushless);
    anglerLeft = new CANSparkFlex(Constants.ShooterConstants.anglerLeftId, brushless);
    anglerRight = new CANSparkFlex(Constants.ShooterConstants.anglerRightId, brushless);
    climberLeft = new CANSparkFlex(Constants.ClimberConstants.leftClimberId, brushless);
    climberRight = new CANSparkFlex(Constants.ClimberConstants.rightClimberId, brushless);

    anglerRight.follow(anglerLeft);
    climberRight.follow(climberRight);

    leftIntakeMotor.setSmartCurrentLimit(40);
    rightIntakeMotor.setSmartCurrentLimit(40);
    rightIntakeMotor.burnFlash();
    leftIntakeMotor.burnFlash();
    anglerRight.burnFlash();
    climberRight.burnFlash();

    
    pheonixdrivebase = new PheonixDrivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);
    shooter = new Manipulator(shooterUpper, shooterLower, ShooterConstants.manipulatorConstants);
    intake = new Manipulator(leftIntakeMotor, rightIntakeMotor, IntakeConstants.manipulatorConstants);
    pivot = new Angler(intakeAnglerMotor, intakeAnglerConst, "pivot");
    shooterAngler = new Angler(anglerLeft, shooterAnglerConst, "shooter");
    climber = new Climber(climberLeft, climberRight);
    beambreak = new DigitalInput(9);
    instance = this;
  }

  public PheonixDrivebase getDrivebase() {
    return pheonixdrivebase;
  }

  public LasaVision getVision() {
    return LasaVision.getInstance();
  }

  public Manipulator getShooter() {
    return shooter;
  }

  public Manipulator getIntake() {
    return intake;
  }

  public Climber getClimber() {
    return climber;
  }

  public Angler getPivot() {
    return pivot;
  }

  public Boolean getBeamBreak() {
    return beambreak.get();
  }

  public Angler getShooterAngler() {
    return shooterAngler;
  }
}
