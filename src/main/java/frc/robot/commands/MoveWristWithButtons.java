package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.ErrorCode;

/**
 * Drive command
 */
public class MoveWristWithButtons extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist wrist;
  private final int direction;
  
  /**
   * Creates a new DriveWithController
   *
   * @param subsystem The subsystem used by this command.
   * @param direction 1 = down, 0 = stop, -1 = up
   */
  public MoveWristWithButtons(Wrist wrist, int direction) {
    this.wrist = wrist;
    this.direction = direction;
    addRequirements(wrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.0;
    int position = wrist.getPosition();
    //going down.  Motor speed is positive, but then turns negative to counteract gravity
    if (direction > 0) {
              //3937,3944 - armPostion in top position
        //4018 - straight up
        //4306 - 75deg
        //4619 - 40 deg
        //4768 - 15 deg
        //5098-9,5109 - armPosition in bottom position
      if (position < 4100) {
        speed = .25;
      } else if (position >= 4100 && position < 4306) {
        speed = 0.0;
      } else if (position >= 4306 && position < 4760) {
        speed = -0.1;
      } else if (position >= 4760 && position < 5100) {
        speed = -0.22;
      } else if (position >= 5100) {
        speed = 0.0;
      }
    //going up - motor speed is negative
    } else if (direction < 0) {
      if (position < 4020) {
        speed = 0.0;
      } else if (position >= 4020 && position < 4306) {
        speed = -0.2;
      } else if (position >= 4306) {
        speed = -0.45;
      } 
    } else {
      speed = 0.0;
    }

//    System.out.println("position = " + position + ", speed = " + speed);
    wrist.move(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
