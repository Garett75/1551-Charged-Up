// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake_Motor extends SubsystemBase {
  double intake_cone_speed = Constants.INTAKE_CONE_SPEED;
  double intake_cube_speed = Constants.INTAKE_CUBE_SPEED;
  CANSparkMax m_intake_motor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new Conveyor_Motor. */
  public Intake_Motor() {
    m_intake_motor.enableVoltageCompensation(10);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Run_Intake_Motor_Cube(double intake_cube_speed) {
    m_intake_motor.set(intake_cube_speed);
  }

  public void Run_Intake_Motor_Cone(double intake_cone_speed) {
    m_intake_motor.set(intake_cone_speed);

  }
}