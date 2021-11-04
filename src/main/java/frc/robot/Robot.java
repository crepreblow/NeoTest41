/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.PowerCell;
import frc.robot.subsystems.DriveSub;
import frc.robot.commands.AutoMode;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  public static PowerCell powercell = new PowerCell();
  public static DriveSub drivesub = new DriveSub();
  public static Climb climb = new Climb();
  public static Pneumatics pneumatics = new Pneumatics();
  public static AutoMode autoCommand = new AutoMode();
  public static OI oi = new OI();

  public double startTime;

  boolean trainstop = true;

  public CANSparkMax m_leftMotor;
  public CANSparkMax m_leftMotor2;
  public CANSparkMax m_leftMotor3;
  public CANSparkMax m_rightMotor;
  public CANSparkMax m_rightMotor2;
  public CANSparkMax m_rightMotor3;

  // Limelight
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private boolean Opstick = true;

  @Override
  public void robotInit() {

    // startTime = Timer.getFPGATimestamp();

    CameraServer.getInstance().startAutomaticCapture("cam0", 0);
    CameraServer.getInstance().startAutomaticCapture("cam1", 1);

    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax
     * object
     * 
     * The CAN ID, which can be configured using the SPARK MAX Client, is passed as
     * the first parameter
     * 
     * The motor type is passed as the second parameter. Motor type can either be:
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
     * 
     * The example below initializes four brushless motors with CAN IDs 1 and 2.
     * Change these parameters to match your setup
     */

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters in the SPARK MAX to their factory default state. If no argument is
     * passed, these parameters will not persist between power cycles
     */
    // m_leftMotor.restoreFactoryDefaults();
    // m_rightMotor.restoreFactoryDefaults();

  }

  public void autonomousInit() {
    if (autoCommand != null)
      autoCommand.start();
  }

  public void autonomousPeriodic() {

    // double time = Timer.getFPGATimestamp();

    /*
     * if(time > 3){
     * 
     * m_rightMotor.set(0.3); m_rightMotor2.set(0.3); m_rightMotor3.set(0.3);
     * m_leftMotor.set(0.3); m_leftMotor2.set(0.3); m_leftMotor3.set(0.3);
     * 
     * }
     * 
     * else {
     * 
     * m_rightMotor.set(0); m_rightMotor2.set(0); m_rightMotor3.set(0);
     * m_leftMotor.set(0); m_leftMotor2.set(0); m_leftMotor3.set(0);
     * 
     * }
     */

    Scheduler.getInstance().run();

    Update_Limelight_Tracking();
    double steer = (OI.Stick.getRawAxis(1));// OI
    double drive = (OI.Stick.getRawAxis(4));// m_leftstick,m_rightstick
    boolean auto = OI.OpStick.getRawButton(2);
    steer *= 1;
    drive *= 1;
    if (auto) {
      if (m_LimelightHasValidTarget) {
        drivesub.DrivetrainArcade(m_LimelightDriveCommand, m_LimelightSteerCommand);
      } else {
        drivesub.DrivetrainArcade(0.0, 0.0);// 0.0
      }
    } else {
      // Drivesubsystem.Drivetrain(drive,steer);
    }

    double Kp = 0.03;

  }

  private void Update_Limelight_Tracking() {
    final double STEER_K = 0.04; // how hard to turn toward//.3
    final double DRIVE_K = 0.85; // how hard to drive fwd//.85
    final double DESIRED_TARGET_AREA = 2.1; // Area of the target when2.1
    final double MAX_DRIVE = 0.6; // Simple speed limit so we/6

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 0.5) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }
    m_LimelightHasValidTarget = true;
    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    // don't let the robot drive too fast into the goal
    if (drive_cmd >= MAX_DRIVE)
      ;
    {
      drive_cmd = MAX_DRIVE;
    }
    if (drive_cmd <= -MAX_DRIVE)
      ;
    {
      drive_cmd = -MAX_DRIVE;
    }

    m_LimelightDriveCommand = drive_cmd;
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // m_myRobot.tankDrive(-m_leftStick.getRawAxis(1), -m_rightStick.getRawAxis(1));

    // Limelight
    Update_Limelight_Tracking();
    double steer = (OI.Stick.getRawAxis(1));// m_rightstick,m_leftstick
    double drive = (OI.Stick.getRawAxis(4));
    boolean auto = OI.OpStick.getRawButton(2);
    steer *= 1;
    drive *= 1;
    if (auto) {
      if (m_LimelightHasValidTarget) {
        drivesub.DrivetrainArcade(m_LimelightDriveCommand, m_LimelightSteerCommand);
      } else {
        drivesub.DrivetrainArcade(0.0, 0.0);
      }
    } else {
      // DriveSub.Drivetrain(drive,steer);
    }

    /*
     * if (Opstick){
     * 
     * drivesub.DrivetrainArcade(0, 0);
     * 
     * 
     * }
     */

    double Kp = 0.03;

  }

}
