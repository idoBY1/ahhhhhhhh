package frc.robot.commands;


import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ReverseControls extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 private boolean m_stop = false; // this is to stopp or 2 not stoop


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseControls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chassis.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Chassis.setReversed(!Chassis.getReversed());
    m_stop = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stop = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished() || m_stop;
  }
}
