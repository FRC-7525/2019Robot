package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drive command
 */
public class MoveWristWithTriggers extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist wrist;
  private final DoubleSupplier downSpeed;
  private final DoubleSupplier upSpeed;

  /**
   * Creates a new DriveWithController
   *
   * @param subsystem The subsystem used by this command.
   * @param upSpeed The suplier of the speed with which to raise the wrist
   * @param downSpeed The supplier of the speed with which to lower the wrist
   */
  public MoveWristWithTriggers(Wrist wrist, DoubleSupplier downSpeed, DoubleSupplier upSpeed) {
    this.wrist = wrist;
    this.downSpeed = downSpeed;
    this.upSpeed = upSpeed;
    addRequirements(wrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double downSpeed = this.downSpeed.getAsDouble();
    double upSpeed = this.upSpeed.getAsDouble();
    wrist.move(downSpeed - upSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
