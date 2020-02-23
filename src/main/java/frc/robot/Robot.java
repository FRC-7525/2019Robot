/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;

/* TODO: need these temporarily to finsh the merge */
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private SpeedController topIntake = new WPI_VictorSPX(8);
  private SpeedController bottomIntake = new WPI_VictorSPX(9);
  private SpeedController liftOne = new WPI_TalonSRX(5);
  private SpeedController liftTwo = new WPI_VictorSPX(6);
  private SpeedController liftThree = new WPI_VictorSPX(7);

  private Solenoid drive_Solenoid = new Solenoid(0);

  private DigitalInput wrist_limit = new DigitalInput(1);
  private DigitalInput hatch_ultrasonic1 = new DigitalInput(2);
  private DigitalInput hatch_ultrasonic2 = new DigitalInput(3);
  private DigitalInput ball_sensor = new DigitalInput(4);

  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.putBoolean("arcade", false);
  }

/**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    /* TODO: Need this temporarily while merging into CommandBased */
    SmartDashboard.putBoolean("wrist_limit", wrist_limit.get());
    SmartDashboard.putBoolean("hatch ultrasonic sensor 1", hatch_ultrasonic1.get());
    SmartDashboard.putBoolean("hatch ultrasonic sensor 2", hatch_ultrasonic2.get());
    SmartDashboard.putBoolean("ball sensor",ball_sensor.get());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
        // Cancels all running commands
        CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    //   m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    // } else {
    //   m_robotDrive.stopMotor(); // stop robot
    // }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
      /* TODO: keep here temporarily while migrating to CommandBased */
    // Button A shifts gears
    if (m_robotContainer.getController().getRawButton(1)) {
      drive_Solenoid.set(true);
    }
    else {
      drive_Solenoid.set(false);
    }

    //Pressing down left joystick will intake
    if (m_robotContainer.getController().getRawButton(9)) {
      topIntake.set(-0.75);
      bottomIntake.set(-0.75);
    }
    //Pressing down right joystick will shoot
    else if (m_robotContainer.getController().getRawButton(10)) {
      topIntake.set(0.75);
      bottomIntake.set(0.75);
    }
    else {
      topIntake.set(0);
      bottomIntake.set(0);
    }
    //Pressing the Y button will make lift go up
    if (m_robotContainer.getController().getRawButton(4)) {
      liftOne.set(0.80);
      liftThree.set(0.80);
    }
    //Pressing the B button will make the lift go down
    else if (m_robotContainer.getController().getRawButton(2)) {
      liftOne.set(-0.60);
      liftThree.set(-0.60);
    }
    else {
      liftOne.set(0);
      liftThree.set(0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
