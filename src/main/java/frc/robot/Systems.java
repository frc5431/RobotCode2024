package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.LasaVision;
import frc.robot.subsystems.PheonixDrivebase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Vision;

public class Systems {
  public static Systems instance;
  // private Vision vision;
  private Manipulator intake;
  private Angler pivot;
  private Angler shooterAngler;
  private Manipulator shooter;

  private AnglerConstants intakeAnglerConst;

  private CANSparkFlex shooterUpper; 
  private CANSparkFlex shooterLower; 

  private CANSparkMax leftIntakeMotor;
  private CANSparkMax rightIntakeMotor;
  private CANSparkMax intakeAnglerMotor;

  private MotorType brushless =  MotorType.kBrushless;
  public PheonixDrivebase drivebase2;

  public Systems() {
    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;
    leftIntakeMotor = new CANSparkMax(Constants.IntakeConstants.leftIntakeId, brushless);
    rightIntakeMotor = new CANSparkMax(Constants.IntakeConstants.rightIntakeId, brushless);
    intakeAnglerMotor = new CANSparkMax(Constants.IntakeConstants.anglerId, brushless);
    drivebase2 = new PheonixDrivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);
    // vision = new Vision(this, new PlacedCamera("shooter_camera", Constants.VisionConstants.ShooterCameraPose));


    shooterLower = new CANSparkFlex(Constants.ShooterConstants.botId, brushless);
    shooterUpper = new CANSparkFlex(Constants.ShooterConstants.topId, brushless);
    shooter = new Manipulator(shooterUpper, shooterLower, ShooterConstants.manipulatorConstants);

    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(leftIntakeMotor, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(intakeAnglerMotor, DCMotor.getNeo550(1));
    }

    leftIntakeMotor.setSmartCurrentLimit(30);
    rightIntakeMotor.setSmartCurrentLimit(30);
    rightIntakeMotor.burnFlash();
    leftIntakeMotor.burnFlash();

    intake = new Manipulator(leftIntakeMotor, rightIntakeMotor, IntakeConstants.manipulatorConstants);
    pivot = new Angler(intakeAnglerMotor, intakeAnglerConst, "pivot");
    instance = this;
  }

  public PheonixDrivebase getDrivebase() {
    return drivebase2;
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

  public Angler getPivot() {
    return pivot;
  }

  public Angler getShooterAngler() {
    return shooterAngler;
  }
}
