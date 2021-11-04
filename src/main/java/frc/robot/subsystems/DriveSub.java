/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.Timer;



/**
 * Add your docs here.
 */
public class DriveSub extends Subsystem {
  
  public int rightDeviceID = 4; 
  public int rightDeviceID2 = 5;
  public int rightDeviceID3 = 6;
  public int leftDeviceID = 1;
  public int leftDeviceID2 = 2;
  public int leftDeviceID3 = 3;

  public CANSparkMax m_rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax m_rightMotor2 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax m_rightMotor3 = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax m_leftMotor3 = new CANSparkMax(3, MotorType.kBrushless); 
  
  SpeedControllerGroup Left = new SpeedControllerGroup(m_leftMotor,m_leftMotor2,m_leftMotor3);
  SpeedControllerGroup Right = new SpeedControllerGroup(m_rightMotor,m_rightMotor2,m_rightMotor3);

  DifferentialDrive m_myRobot = new DifferentialDrive(Left, Right);

  public void Drivetrain(double Lspeed,double Rspeed){
    
    m_myRobot.tankDrive(Lspeed,Rspeed);//Lspeed,Rspeed
 
  }
  public void DrivetrainArcade(double forward,double turn){
    
    m_myRobot.arcadeDrive(forward, turn);
 
  }
  
  public void DriveStop(){
    
   // m_myRobot.tankDrive(0, 0);
    m_myRobot.arcadeDrive(0, 0);
  
  }
  
  public void Halfspeed(){

    m_myRobot.arcadeDrive(0.5, 0.5);

  }


  @Override
  public void initDefaultCommand() {
    
    setDefaultCommand(new Drive());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void AutoDrive(){

    m_myRobot.arcadeDrive(.6, 0);
    Timer.delay(3);
    m_myRobot.arcadeDrive(0, 0);
  //Timer.delay(1);
    /*m_myRobot.arcadeDrive(.5, -.5);
    Timer.delay(3);
    m_myRobot.arcadeDrive(0, 0);*/

   // if(Robot.  )

  }



}
