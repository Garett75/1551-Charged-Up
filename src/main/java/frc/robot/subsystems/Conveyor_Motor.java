// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor_Motor extends SubsystemBase {
  double conveyor_speed = Constants.CONVEYOR_SPEED;
  CANSparkMax m_conveyor_motor = new CANSparkMax(Constants.CONVEYOR_MOTOR_ID, MotorType.kBrushless);
  
  /** Creates a new Conveyor_Motor. */
  public Conveyor_Motor() {
    //m_conveyor_motor.enableVoltageCompensation(11);
    //m_conveyor_motor.getEncoder().setPositionConversionFactor(0.094245);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Run_Conveyor_Motor(double conveyor_speed) {
    m_conveyor_motor.set(conveyor_speed);
  }

  public double getDistance(){
    return m_conveyor_motor.getEncoder().getPosition();
  }
  
  public double get_output(){
    return m_conveyor_motor.get();
  }
}
