// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorPorts;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  // initialize right motors
  private SpeedControllerGroup m_right = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisRightFront), new WPI_VictorSPX(MotorPorts.chassisRightBack));

  // initialize left motors
  private SpeedControllerGroup m_left = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisLeftFront), new WPI_VictorSPX(MotorPorts.chassisLeftBack));

  private static final Chassis m_chassis = new Chassis(); // creates the only instance of Chassis
  
  private static boolean isReversed = true; // variable that checks if the drive direction is reversed

  private Chassis() {}

  public synchronized static Chassis getInstance() {
    return m_chassis;
  }

  // seting the speeds of the motors
  public void setRightMotorsSpeed(double speed) {
    m_right.set(speed * (isReversed ? 1 : -1)); // if isReversed is true this will be equal to (1 * speed), and if isReversed is false this will be equal to (-1 * speed)
  }

  public void setLeftMotorsSpeed(double speed) {
    m_left.set(speed * (isReversed ? 1 : -1)); // same as above but for the left motors
  }

 // sets the direction
 public static void setReversed(boolean isReversed) {
   Chassis.isReversed = isReversed;
 }
// gets the direction
public static boolean getReversed() {
  return Chassis.isReversed;
}




  // tank move (using the speed controllers above)
  public void tankMove(double leftSpeed, double rightSpeed) {
    this.setRightMotorsSpeed(rightSpeed);
    this.setLeftMotorsSpeed(leftSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
