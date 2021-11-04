/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.OI;

public class Drive extends Command {
  public Drive() {
    requires(Robot.drivesub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivesub.DriveStop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.drivesub.Drivetrain(OI.Getleftstick(), OI.Getrightstick());
    Robot.drivesub.DrivetrainArcade(OI.Getleftstick(), OI.Getrightstick());  
    
    


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivesub.DriveStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}