/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory
 */
public class Robot extends TimedRobot {
  // 0 - Left Motor 1
  // 1 - Right Motor 1
  // 2 - Left Motor 2
  // 3 - Right Motor 2
  private SpeedController right1 = new WPI_TalonSRX(0);
  private SpeedController right2 = new WPI_VictorSPX(2);
  private SpeedController left1 = new WPI_TalonSRX(1);
  private SpeedController left2 = new WPI_VictorSPX(3);
  private SpeedController armMotor = new WPI_TalonSRX(4);
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

  
  private final SpeedControllerGroup m_leftControllerGroup = 
    new SpeedControllerGroup(left1, left2);
  private final SpeedControllerGroup m_rightControllerGroup = 
    new SpeedControllerGroup(right1, right2);
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    ((WPI_TalonSRX)armMotor).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
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
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY() * -1, m_stick.getX() * -1);
    // Button A shifts gears
    if (m_stick.getRawButton(1)) {
      drive_Solenoid.set(true);
    }
    else {
      drive_Solenoid.set(false);
    }
    //left bumper button extends the arm
    if (m_stick.getRawButton(5)) {
      armMotor.set(0.75);
    }
    //right bumper button retracts the arm
    else if (m_stick.getRawButton(6) && wrist_limit.get()) {
      armMotor.set(-0.75);
    }
    else {
      armMotor.set(0);
    }
    //Pressing down left joystick will intake
    if (m_stick.getRawButton(9)) {
      topIntake.set(-0.75);
      bottomIntake.set(-0.75);
    }
    //Pressing down right joystick will shoot
    else if (m_stick.getRawButton(10)) {
      topIntake.set(0.75);
      bottomIntake.set(0.75);
    }
    else {
      topIntake.set(0);
      bottomIntake.set(0);
    }
    //Pressing the Y button will make lift go up
    if (m_stick.getRawButton(4)) {
      liftOne.set(0.50);
      liftThree.set(0.50);
    }
    //Pressing the B button will make the lift go down
    else if (m_stick.getRawButton(2)) {
      liftOne.set(-0.50);
      liftThree.set(-0.50);
    }
    else {
      liftOne.set(0);
      liftThree.set(0);
    }
  }
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("wrist_limit", wrist_limit.get());
    SmartDashboard.putBoolean("hatch ultrasonic sensor 1", hatch_ultrasonic1.get());
    SmartDashboard.putBoolean("hatch ultrasonic sensor 2", hatch_ultrasonic2.get());
    SmartDashboard.putBoolean("ball sensor",ball_sensor.get());
    int sensorPosition = ((WPI_TalonSRX)armMotor).getSelectedSensorPosition();
    SmartDashboard.putNumber("armPosition", sensorPosition);
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
