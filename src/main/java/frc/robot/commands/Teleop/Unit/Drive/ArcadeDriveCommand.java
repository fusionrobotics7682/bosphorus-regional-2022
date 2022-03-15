package frc.robot.commands.Teleop.unit.Drive;






import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  final DriveSubsystem driveSubsystem;
  
  /** Creates a new ArcadeDriveCommand. */
  public ArcadeDriveCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArcadeDriveCommand.initialize()");
    driveSubsystem.differentialDrive.setMaxOutput(0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
