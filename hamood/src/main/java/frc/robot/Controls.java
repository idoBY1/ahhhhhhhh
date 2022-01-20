package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Controls {

    // creates joystick variables
    private static Joystick m_rightJoystick, m_leftJoystick, m_operatorJoystick; 
    private static JoystickButton m_reverseButton;



  // initialize \\
  //-------------------------------------------------------------------------------------------------------------\\
  public static void init() {
    // Joysticks \\
    m_rightJoystick = new Joystick(Constants.JoystickPorts.rightChassisJS);
    m_leftJoystick = new Joystick(Constants.JoystickPorts.leftChassisJS);
    m_operatorJoystick = new Joystick(Constants.JoystickPorts.operatorJS);

    // Buttons \\
    m_reverseButton = new JoystickButton(m_rightJoystick, Constants.ButtonPorts.LEFT_JS_REVERSE_CONTROLS);
  }
  
  //-----------------------------------------------------------------------------\\
  // Joysticks \\
  // gets the left joystick
  public static Joystick getM_leftJoystick() {
      return m_leftJoystick;
  }

  // gets the right joystick
  public static Joystick getM_rightJoystick() {
      return m_rightJoystick;
  }

  // gets the operator joystick 
  public static Joystick getM_operatorJoystick() {
      return m_operatorJoystick;
  }

  //--------------------------------------------------------------------------------\\

  // Buttons \\
  // gets the joystick button
  public static JoystickButton getM_reverseButton() {
      return m_reverseButton;
  }

  //--------------------------------------------------------------------------------\\
  
  // uses tank-drive whil in periodic mode
  public static void movePeriodic() { 
    double leftSpeed = m_leftJoystick.getY() * Constants.MathConsts.DONT_KILL_THE_MOTORS;
    double rightSpeed = m_rightJoystick.getY() * Constants.MathConsts.DONT_KILL_THE_MOTORS;
    Chassis.getInstance().tankMove(leftSpeed,rightSpeed);
  }

  // all the commands that are linked to buttons are initialized here \\
  public static void commandsPeriodic() {

  }
}