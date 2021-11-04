/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.ejml.dense.block.MatrixMult_DDRB;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Intaking;
import frc.robot.commands.Outtaking;
import frc.robot.commands.Stop;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.OutTaking2;
import frc.robot.commands.Climbing;
import frc.robot.commands.Intaking2;
import frc.robot.commands.PneuOpen;
import frc.robot.commands.PneuOpen2;
import frc.robot.commands.PneuOpen3;
import frc.robot.commands.PneuClosed;
import frc.robot.commands.PneuClosed2;
import frc.robot.commands.PneuClosed3;
import frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Halfspeed;

/**
 * Add your docs here.
 */
public class OI {

  public double startTime = Timer.getFPGATimestamp();

  public static Joystick m_leftStick = new Joystick(0); // 10volts
  public static Joystick m_rightStick = new Joystick(1); // 3000rpm
  public static Joystick OpStick = new Joystick(2);
  public static XboxController Stick = new XboxController(1);
  public static XboxController Stick2 = new XboxController(0);

  public static double Getleftstick() {
    return (m_leftStick.getRawAxis(1));// -m_leftStick m_rightStick

  }

  public static double Getrightstick() {
    return (m_rightStick.getRawAxis(4));// -m_rightStick m_rightStick
  }

  Button Btn_Halfspeed = new JoystickButton(Stick, 9);
  Button Btn_Halfspeedturn = new JoystickButton(Stick, 10);
  Button Btn_Intaking = new JoystickButton(Stick, 5);
  Button Btn_Outtaking = new JoystickButton(Stick, 6);
  Button Btn_Limelight = new JoystickButton(Stick, 2);

  Button Btn_Shooter = new JoystickButton(OpStick, 1);
  Button Btn_Intaking2 = new JoystickButton(OpStick, 5);
  Button Btn_Outtaking2 = new JoystickButton(OpStick, 6);
  Button Btn_Climbing = new JoystickButton(OpStick, 12);
  Button Btn_PneuOpen = new JoystickButton(OpStick, 7);// Button for extending climbing
  Button Btn_PneuOpen2 = new JoystickButton(OpStick, 8);// Button for extending outer intake
  // Button Btn_PneuOpen3 = new JoystickButton(OpStick, 11);
  Button Btn_PneuClosed = new JoystickButton(OpStick, 9);// Button for bringing down climb
  Button Btn_PneuClosed2 = new JoystickButton(OpStick, 10);// Button for bringing in outer intake
  // Button Btn_PneuClosed3 = new JoystickButton(OpStick, 12);

  public OI() {

    Btn_Intaking.whenPressed(new Intaking());
    Btn_Intaking.whenInactive(new Stop());
    Btn_Intaking2.whenPressed(new Intaking2());
    Btn_Intaking2.whenInactive(new Stop());
    Btn_Outtaking.whenPressed(new Outtaking());
    Btn_Outtaking.whenInactive(new Stop());
    Btn_Outtaking2.whenPressed(new OutTaking2());
    Btn_Outtaking2.whenInactive(new Stop());
    Btn_Shooter.whenPressed(new Shooter());
    Btn_Shooter.whenInactive(new Stop());
    Btn_Climbing.whenPressed(new Climbing());
    Btn_Climbing.whenInactive(new Stop());
    Btn_PneuOpen.whenPressed(new PneuOpen());
    Btn_PneuOpen.whenInactive(new Stop());
    Btn_PneuOpen2.whenPressed(new PneuOpen2());
    Btn_PneuOpen2.whenInactive(new Stop());
    // Btn_PneuOpen3.whenPressed(new PneuOpen3());
    // Btn_PneuOpen3.whenInactive(new Stop());
    Btn_PneuClosed.whenPressed(new PneuClosed());
    Btn_PneuClosed.whenInactive(new Stop());
    Btn_PneuClosed2.whenPressed(new PneuClosed2());
    Btn_PneuClosed2.whenInactive(new Stop());
    // Btn_PneuClosed3.whenPressed(new PneuClosed3());
    // Btn_PneuClosed3.whenInactive(new Stop());
    Btn_Halfspeed.whenPressed(new Halfspeed());
    Btn_Halfspeed.whenInactive(new Stop());
    Btn_Halfspeedturn.whenPressed(new Halfspeed());
    Btn_Halfspeedturn.whenInactive(new Stop());

  }

}