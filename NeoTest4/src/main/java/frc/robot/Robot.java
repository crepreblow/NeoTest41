/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.PowerCell;
import frc.robot.subsystems.DriveSub;
import frc.robot.commands.OutTaking2;
import frc.robot.commands.Outtaking;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANEncoder;

public class Robot extends TimedRobot {
  
  public static PowerCell powercell = new PowerCell();
  public static DriveSub drivesub = new DriveSub();
  public static Climb climb = new Climb();
  public static Pneumatics pneumatics = new Pneumatics();
  public static OI oi = new OI();
  
  private double startTime;
  
  /*private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private Joystick OpStick;*/
  
  /*public int rightDeviceID = 4; 
  public int rightDeviceID2 = 5;
  public int rightDeviceID3 = 6;
  public int leftDeviceID = 1;
  public int leftDeviceID2 = 2;
  public int leftDeviceID3 = 3;*/
 
  public CANSparkMax m_leftMotor;
  public CANSparkMax m_leftMotor2;
  public CANSparkMax m_leftMotor3;
  public CANSparkMax m_rightMotor; 
  public CANSparkMax m_rightMotor2;
  public CANSparkMax m_rightMotor3;
  
  /*public CANSparkMax m_rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax m_rightMotor2 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax m_rightMotor3 = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax m_leftMotor3 = new CANSparkMax(3, MotorType.kBrushless); */
  
  //Limelight
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  
  @Override
  public void robotInit() {
  
    //m_leftMotor.configselectedFeedbackSensor(FeedbackDevice.CANEncoder, 0, 10);
    
    CameraServer.getInstance().startAutomaticCapture("cam0",0);
    CameraServer.getInstance().startAutomaticCapture("cam1",1);
  
    /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
   


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    //m_leftMotor.restoreFactoryDefaults();
    //m_rightMotor.restoreFactoryDefaults();

    /*SpeedControllerGroup Left = new SpeedControllerGroup(m_leftMotor,m_leftMotor2,m_leftMotor3);
    SpeedControllerGroup Right = new SpeedControllerGroup(m_rightMotor,m_rightMotor2,m_rightMotor3);

    m_myRobot = new DifferentialDrive(Left, Right);*/

    /*m_leftStick = new Joystick(2);
    m_rightStick = new Joystick(0);
    OpStick = new Joystick(1);*/
  
  }

  public void autonomousInit(){

     startTime = Timer.getFPGATimestamp();
     
  
    }
  
  public void autonomousPeriodic() {
    
    double time = Timer.getFPGATimestamp();
    
    Outtaking.set(0.5);
    OutTaking2.set(0.5);
    Timer.delay(4);
    


    if(time > 2){
      m_rightMotor.set(-0.5);
      m_rightMotor2.set(-0.5);
      m_rightMotor3.set(-0.5);
      m_leftMotor.set(-0.5);
      m_leftMotor2.set(-0.5);
      m_leftMotor3.set(-0.5);
      
    }
     else {
      m_rightMotor.set(0);
      m_rightMotor2.set(0);
      m_rightMotor3.set(0);
      m_leftMotor.set(0);
      m_leftMotor2.set(0);
      m_leftMotor3.set(0);

    }
    
    if (time < 2){
      m_rightMotor.set(0);
      m_rightMotor2.set(0);
      m_rightMotor3.set(0);
      m_leftMotor.set(-0.5);
      m_leftMotor2.set(-0.5);
      m_leftMotor3.set(-0.5);
      
      }

    else if (time > 3) {
      m_rightMotor.set(0);
      m_rightMotor2.set(0);
      m_rightMotor3.set(0);
      m_leftMotor.set(0);
      m_leftMotor2.set(0);
      m_leftMotor3.set(0);

      }
    
    
    Scheduler.getInstance().run();
   
    Update_Limelight_Tracking();
    double steer = (OI.m_rightStick.getRawAxis(1));
    double drive = (OI.m_leftStick.getRawAxis(1));
    boolean auto = OI.OpStick.getRawButton(2);
    steer *= 1;
    drive *= 1;
    if (auto)
    {
      if (m_LimelightHasValidTarget)
      {
       drivesub.Drivetrain(m_LimelightDriveCommand,m_LimelightSteerCommand);
      }
      else
      {
        drivesub.Drivetrain(0.0,0.0);//0.0
      }
    }
    else
    {
    //Drivesubsystem.Drivetrain(drive,steer);
    }
 
    double Kp = 0.03;
 
    }

    
    private void Update_Limelight_Tracking() {
    final double STEER_K = 0.3; // how hard to turn toward//3
    final double DRIVE_K = 0.85; // how hard to drive fwd//.85
    final double DESIRED_TARGET_AREA = 15.0; // Area of the target when
    final double MAX_DRIVE = -0.6; // Simple speed limit so we/6
    
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    if (tv < 0.5)
    {
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
    if (drive_cmd >= MAX_DRIVE);
    {
    drive_cmd = MAX_DRIVE;
    }
    if(drive_cmd <= -MAX_DRIVE);{
      drive_cmd = -MAX_DRIVE;
    }
    
    m_LimelightDriveCommand = drive_cmd;
  }

  
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
   // m_myRobot.tankDrive(-m_leftStick.getRawAxis(1), -m_rightStick.getRawAxis(1));
  
  //Limelight
  Update_Limelight_Tracking();
  double steer = (OI.m_rightStick.getRawAxis(1));
  double drive = (OI.m_leftStick.getRawAxis(1));
  boolean auto = OI.OpStick.getRawButton(2);
  steer *= 1;
  drive *= 1;
  if (auto)
  {
    if (m_LimelightHasValidTarget)
    {
      drivesub.Drivetrain(m_LimelightDriveCommand,m_LimelightSteerCommand);
    }
    else
    {
      drivesub.Drivetrain(0.0,0.0);
    }
  }
  else
  {
    //DriveSub.Drivetrain(drive,steer);
  }

  double Kp = 0.03;

  }
  
  }
