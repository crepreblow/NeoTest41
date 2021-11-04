/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
import frc.robot.commands.Intaking3;
import frc.robot.commands.Intaking4;

/**
 * Add your docs here.
 */
public class OI {

    public static Joystick m_leftStick = new Joystick(1);
    public static Joystick m_rightStick = new Joystick(2);
    public static Joystick OpStick = new Joystick(0);

    public static double Getleftstick(){
        return(-m_leftStick.getRawAxis(1));
      
      }
      public static double Getrightstick(){
        return(-m_rightStick.getRawAxis(1));
      }

      Button Btn_Intaking = new JoystickButton(OpStick, 3);
      Button Btn_Intaking2 = new JoystickButton(OpStick, 3);
      Button Btn_Intaking3 = new JoystickButton(OpStick, 4);
      Button Btn_Intaking4 = new JoystickButton(OpStick, 4);
      Button Btn_Outtaking = new JoystickButton(OpStick, 5);
      Button Btn_Outtaking2 = new JoystickButton(OpStick, 6);
      Button Btn_Limelight = new JoystickButton(OpStick, 2);
      Button Btn_Climbing = new JoystickButton(OpStick, 11);
      Button Btn_PneuOpen = new JoystickButton(OpStick, 7);
      Button Btn_PneuOpen2 = new JoystickButton(OpStick,7);
      Button Btn_PneuOpen3 = new JoystickButton(OpStick, 9);
      Button Btn_PneuClosed = new JoystickButton(OpStick, 8);
      Button Btn_PneuClosed2 = new JoystickButton(OpStick, 8);
      Button Btn_PneuClosed3 = new JoystickButton(OpStick, 10);      
      
      public OI(){
        Btn_Intaking.whenPressed(new Intaking());
        Btn_Intaking.whenInactive(new Stop());
        Btn_Intaking2.whenPressed(new Intaking2());
        Btn_Intaking2.whenInactive(new Stop());
        Btn_Intaking3.whenPressed(new Intaking3());
        Btn_Intaking3.whenInactive(new Stop());
        Btn_Intaking4.whenPressed(new Intaking4());
        Btn_Intaking4.whenInactive(new Stop());
        Btn_Outtaking.whenPressed(new Outtaking());
        Btn_Outtaking.whenInactive(new Stop());
        Btn_Outtaking2.whenPressed(new OutTaking2());
        Btn_Outtaking2.whenInactive(new Stop());
        Btn_Climbing.whenPressed(new Climbing());
        Btn_Climbing.whenInactive(new Stop());
        Btn_PneuOpen.whenPressed(new PneuOpen());
        Btn_PneuOpen.whenInactive(new Stop());
        Btn_PneuOpen2.whenPressed(new PneuOpen2());
        Btn_PneuOpen2.whenInactive(new Stop());
        Btn_PneuOpen3.whenPressed(new PneuOpen3());
        Btn_PneuOpen3.whenInactive(new Stop());
        Btn_PneuClosed.whenPressed(new PneuClosed());
        Btn_PneuClosed.whenInactive(new Stop());
        Btn_PneuClosed2.whenPressed(new PneuClosed2());
        Btn_PneuClosed2.whenInactive(new Stop());
        Btn_PneuClosed3.whenPressed(new PneuClosed3());
        Btn_PneuClosed3.whenInactive(new Stop());
        






}


}

