

package frc.robot.commands.Autonomous.TimerBased;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoThrowBallCommand extends CommandBase {

  Timer timer = new Timer();

  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  
  DigitalInput frontLimitSwitch;

  /** Creates a new JoystickThrowBallCommandd. */
  public AutoThrowBallCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    timer.reset();
    timer.start();
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("JOYSTICK THROW BALL COMMAND INITIALIZED !!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() <= 2){
      shooterSubsystem.shoot();
    }
    else if(timer.get() <= 4){
      feederSubsystem.getIn();
    }
    else{
      shooterSubsystem.stopMotor();
      feederSubsystem.stopMotor();
    }      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(frontLimitSwitch.get() == true) {
      return true;
    }
    return false;
  }
}
