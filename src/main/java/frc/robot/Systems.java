package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.AnglerConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;


public class Systems {

  private Vision vision;
  private Intake intake;
  private Angler pivot;
  private Angler shooterAngler;
  private Drivebase drivebase;

  private AnglerConstants intakeAnglerConst;

  private CANSparkMax intakeMotor;
  private CANSparkMax intakeAnglerMotor;

  public Systems() {
    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeId, MotorType.kBrushless);
    intakeAnglerMotor = new CANSparkMax(Constants.IntakeConstants.anglerId, MotorType.kBrushless);

    drivebase = new Drivebase();

    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(intakeAnglerMotor, DCMotor.getNeo550(1));
    }


    // vision = new Vision();
    

    intake = new Intake(intakeMotor);
    pivot = new Angler(intakeAnglerMotor, intakeAnglerConst, "pivot");
  }

  public Drivebase getDrivebase() {
    return drivebase;
  }

  public Vision getVision() {
    return vision;
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
