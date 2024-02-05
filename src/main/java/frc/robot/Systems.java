package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.AnglerConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
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

  private CANSparkMax intakeMotor;
  private CANSparkMax intakeAnglerMotor;

  private MotorType brushless =  MotorType.kBrushless;

  public Systems() {
    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeId, brushless);
    intakeAnglerMotor = new CANSparkMax(Constants.IntakeConstants.anglerId, brushless);


    shooterLower = new CANSparkFlex(Constants.ShooterConstants.botId, brushless);
    shooterUpper = new CANSparkFlex(Constants.ShooterConstants.topId, brushless);
    shooter = new Shooter(shooterUpper, shooterLower);
    drivebase = new Drivebase();

    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(intakeAnglerMotor, DCMotor.getNeo550(1));
    }

    // vision = new Vision();
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.burnFlash();

    intake = new Intake(intakeMotor);
    pivot = new Angler(intakeAnglerMotor, intakeAnglerConst, "pivot");
  }

  public Drivebase getDrivebase() {
    return drivebase;
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
