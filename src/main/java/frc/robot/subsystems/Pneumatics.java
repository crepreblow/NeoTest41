/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  
  public DoubleSolenoid S1 = new DoubleSolenoid(4,5);//Climb
  public DoubleSolenoid S2 = new DoubleSolenoid(2,3);//OuterIntake
  public DoubleSolenoid S3 = new DoubleSolenoid(0,1);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
