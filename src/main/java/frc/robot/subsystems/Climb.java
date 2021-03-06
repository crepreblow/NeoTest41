/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climb extends Subsystem {
  
 public int climbDeviceID = 9;
 
 public CANSparkMax climbmotor = new CANSparkMax(9,MotorType.kBrushless);
  
  public void Climbing(){
    
    climbmotor.set(-0.5);//-1 counterclockwise, 1 clockwise
  
  }
  public void Stop(){
    
    climbmotor.set(0);
  
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
