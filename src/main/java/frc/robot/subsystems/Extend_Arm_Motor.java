// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extend_Arm_Motor extends SubsystemBase {
  double extend_arm_speed = Constants.EXTEND_ARM_SPEED;
  public static CANSparkMax m_extend_arm_motor = new CANSparkMax(Constants.EXTEND_ARM_MOTOR_ID, MotorType.kBrushless);
  public static RelativeEncoder m_encoder = m_extend_arm_motor.getEncoder();
  public DigitalInput limit_switch = new DigitalInput(Constants.RETRACT_LIMIT_SWITCH_ID);

  /** Creates a new Conveyor_Motor. */
  public Extend_Arm_Motor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Run_Extend_Arm_Motor(double extend_arm_speed) {
    m_extend_arm_motor.set(extend_arm_speed);
  }

  public void Run_Arm_Extend_Motor_With_Joystick(double extend_joystick_speed) {
    m_extend_arm_motor.set(extend_joystick_speed);
  }
}