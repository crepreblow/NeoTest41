/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANPIDController;


/**
 * Add your docs here.
 */
public class PowerCell extends Subsystem {
  
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public CANSparkMax OuttakeMotor = new CANSparkMax(7,MotorType.kBrushless);
  public CANSparkMax OuttakeMotor2 = new CANSparkMax(8,MotorType.kBrushless);
  public VictorSP IntakeMotor = new VictorSP(3);//Outer Intake
  public VictorSP IntakeMotor2 = new VictorSP(4);//Outer Intake
  public VictorSP IntakeMotor3 = new VictorSP(1);//Inner Intake
  public VictorSP IntakeMotor4 = new VictorSP(2);//Inner Intake
  
  public CANEncoder encoder = OuttakeMotor.getEncoder(EncoderType.kQuadrature, 42);
  public CANEncoder encoder2 = OuttakeMotor2.getEncoder(EncoderType.kQuadrature, 42);
  
  public CANPIDController pidcontroller = OuttakeMotor.getPIDController();
  
    /*pidcontroller.setFeedbackDevice(encoder);
  
    
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;*/
    
    
    
    public void Intaking(){
    
    IntakeMotor.set(1);
    IntakeMotor2.set(1);
  
  }
    public void Intaking2(){

      IntakeMotor3.set(-1);
      IntakeMotor4.set(-1);
    
    }

    public void Stop(){
    
    IntakeMotor.set(0);
    IntakeMotor2.set(0);
    IntakeMotor3.set(0);
    IntakeMotor4.set(0);
    OuttakeMotor.set(0);
    OuttakeMotor2.set(0);
    
  
  }
  public void Outtaking(){
    
    IntakeMotor.set(-1);
    IntakeMotor2.set(-1);
  
  }
  public void Outtaking2(){

    IntakeMotor3.set(1);
    IntakeMotor4.set(1);

  }
  public void Outtaking3(){

    OuttakeMotor.set(1);
    OuttakeMotor2.set(1);

  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
