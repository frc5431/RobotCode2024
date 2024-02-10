package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PheonixDrivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Systems {

  private Vision vision;
  private Intake intake;
  private Angler pivot;
  private Angler shooterAngler;
  private Drivebase drivebase;
  private Shooter shooter;

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


    shooterLower = new CANSparkFlex(Constants.ShooterConstants.botId, brushless);
    shooterUpper = new CANSparkFlex(Constants.ShooterConstants.topId, brushless);
    shooter = new Shooter(shooterUpper, shooterLower);
    drivebase2 = new PheonixDrivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);

    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(leftIntakeMotor, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(intakeAnglerMotor, DCMotor.getNeo550(1));
    }

    // vision = new Vision();
    leftIntakeMotor.setSmartCurrentLimit(40);
    rightIntakeMotor.setSmartCurrentLimit(40);
    rightIntakeMotor.burnFlash();
    leftIntakeMotor.burnFlash();

    intake = new Intake(leftIntakeMotor, rightIntakeMotor);
    pivot = new Angler(intakeAnglerMotor, intakeAnglerConst, "pivot");
  }

  public PheonixDrivebase getDrivebase() {
    return drivebase2;
  }

  public Vision getVision() {
    return vision;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Intake getIntake() {
    return intake;
  }

  public Angler getPivot() {
    return pivot;
  }

  public Angler getShooterAngler() {
    return shooterAngler;
  }
}
