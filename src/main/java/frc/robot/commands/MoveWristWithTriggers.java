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
  private final DoubleSupplier upSpeed;
  private final DoubleSupplier downSpeed;

  /**
   * Creates a new DriveWithController
   *
   * @param subsystem The subsystem used by this command.
   * @param upSpeed The suplier of the speed with which to raise the wrist
   * @param downSpeed The supplier of the speed with which to lower the wrist
   */
  public MoveWristWithTriggers(Wrist wrist, DoubleSupplier upSpeed, DoubleSupplier downSpeed) {
    this.wrist = wrist;
    this.upSpeed = upSpeed;
    this.downSpeed = downSpeed;
    addRequirements(wrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upSpeed = this.upSpeed.getAsDouble();
    double downSpeed = this.downSpeed.getAsDouble();
    wrist.move(upSpeed - downSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
