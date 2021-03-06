/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.MoveWristWithTriggers;
import frc.robot.commands.MoveWristWithButtons;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drivetrain = new DriveTrain();
  private final Wrist wrist = new Wrist();
  
  // The driver's controller
  XboxController gameController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DriveWithController(
            m_drivetrain,
            () -> -gameController.getY(GenericHID.Hand.kLeft),
            () -> gameController.getX(GenericHID.Hand.kRight)
        )
    );
    wrist.setDefaultCommand(
      new MoveWristWithTriggers(
        wrist,
        () -> gameController.getTriggerAxis(GenericHID.Hand.kLeft),
        () -> gameController.getTriggerAxis(GenericHID.Hand.kRight)
      )
    );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // Grab the hatch when the 'A' button is pressed.
        new JoystickButton(gameController, Button.kBumperLeft.value)
          .whenHeld(new MoveWristWithButtons(wrist, 1));
        new JoystickButton(gameController, Button.kBumperRight.value)
          .whenHeld(new MoveWristWithButtons(wrist, -1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  //}

  /* TODO: only here temporarily while transitioning to CommandBased */
  public XboxController getController() {
    return gameController;
  }
}
