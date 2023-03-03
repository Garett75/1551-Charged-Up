// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Bottom_Winch_Motor extends SubsystemBase {
  double bottom_winch_speed = Constants.BOTTOM_WINCH_SPEED;
  CANSparkMax m_bottom_winch_motor = new CANSparkMax(Constants.BOTTOM_WINCH_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new Conveyor_Motor. */
  public Bottom_Winch_Motor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Run_Bottom_Winch_Motor(double bottom_winch_speed) {
    m_bottom_winch_motor.set(bottom_winch_speed);
  }
}