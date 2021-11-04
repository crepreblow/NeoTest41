/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class PowerCell extends Subsystem {
  
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public CANEncoder encoder;
  public CANEncoder encoder2;
  public CANPIDController m_pidController;
  public CANPIDController m_pidController2;

  public CANSparkMax OuttakeMotor = new CANSparkMax(7,MotorType.kBrushless);
  public CANSparkMax OuttakeMotor2 = new CANSparkMax(8,MotorType.kBrushless);
  public VictorSP IntakeMotor = new VictorSP(1);
  public VictorSP IntakeMotor3 = new VictorSP(2);
  public VictorSP IntakeMotor4 = new VictorSP(3);

  public void OutputVoltage() {
    OuttakeMotor.getBusVoltage();
    SmartDashboard.putNumber("Output Voltage", OuttakeMotor.getBusVoltage());
    OuttakeMotor2.getBusVoltage();
    SmartDashboard.putNumber("Output Voltage", OuttakeMotor2.getBusVoltage());
    OuttakeMotor.setVoltage(10);
    OuttakeMotor2.setVoltage(10);
  }

  public void Encoders()  {

    encoder = OuttakeMotor.getEncoder(EncoderType.kQuadrature, 4096);
    encoder2 = OuttakeMotor2.getEncoder(EncoderType.kQuadrature, 4096);
    
    m_pidController = OuttakeMotor.getPIDController();
    m_pidController2 = OuttakeMotor2.getPIDController();
  
    m_pidController.setFeedbackDevice(encoder);
    m_pidController2.setFeedbackDevice(encoder2);

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; //remove all other pid values and tweek only feed-forward until RPM reaches close to setpoint
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    
  }
  
  public void EncoderRead(){

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    m_pidController.setReference(rotations, ControlType.kPosition);
    m_pidController2.setReference(rotations, ControlType.kPosition);

    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", encoder.getPosition());
    SmartDashboard.putNumber("ProcessVariable", encoder2.getPosition());

  }


  public void Intaking(){
    
    IntakeMotor.set(0.75);//OuterIntake Motor
    
  
  }
  public void Intaking2(){

    IntakeMotor3.set(-1);//InnerIntake Motors
    IntakeMotor4.set(-1);

  }
  
  
  
  public void Stop(){
    
    IntakeMotor.set(0);
    IntakeMotor3.set(0);
    IntakeMotor4.set(0);
    OuttakeMotor.set(0);
    OuttakeMotor2.set(0);
  
  }
  
  public void Outtaking(){

    IntakeMotor.set(-0.75);//Reverse
    

  }
 
  public void Outtaking2(){

    IntakeMotor3.set(1);//Reverse
    IntakeMotor4.set(1);

  }

  public void Shooter(){
    
    OuttakeMotor.set(-1);
    OuttakeMotor2.set(1);


  }
  
  public void outputShooter(){

  System.out.println();

  }


  public void AutoPowerCell(){

    //Timer.delay(1);
    OuttakeMotor.set(-1);
    OuttakeMotor2.set(1);
    Timer.delay(1);
    IntakeMotor3.set(-1);//InnerIntake Motors
    IntakeMotor4.set(-1);
    Timer.delay(3);
    IntakeMotor3.set(0);//InnerIntake Motors
    IntakeMotor4.set(0);
    OuttakeMotor.set(0);
    OuttakeMotor2.set(0);
    
  }
  
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
