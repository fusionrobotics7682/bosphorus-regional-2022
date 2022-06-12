package frc.robot.commands.Teleop.Binary;




import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithFeederCommand extends CommandBase {

  /*DigitalInput frontLimitSwitch;
  DigitalInput backLimitSwitch;*/

  private FeederSubsystem feederSubsystem;
  private ShooterSubsystem shooterSubsystem;

  /** Creates a new ThrowBallCommand. */
  public ShootWithFeederCommand(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(feederSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("THROW BALL COMMAND INITIALIZED !!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.getIn();
    shooterSubsystem.shoot(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    System.out.println("THROW BALL COMMAND IS FINISHED WORKED !!!");
    if(frontLimitSwitch.get() == false && backLimitSwitch.get() == false){
      return true;
    }
    return false;
  }*/
    return false;
  }
}
