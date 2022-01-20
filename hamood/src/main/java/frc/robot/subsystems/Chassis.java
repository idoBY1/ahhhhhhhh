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

  private Chassis() {}

  public synchronized static Chassis getInstance() {
    return m_chassis;
  }

  // seting the speeds of the motors
  public void setRightMotorsSpeed(double speed) {
    m_right.set(speed); 
  }

  public void setLeftMotorsSpeed(double speed) {
    m_left.set(speed); 
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
