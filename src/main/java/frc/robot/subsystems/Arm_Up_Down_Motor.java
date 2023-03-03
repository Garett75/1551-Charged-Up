// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm_Up_Down_Motor extends SubsystemBase {
  double up_down_speed = Constants.ARM_UP_DOWN_SPEED;
  public static CANSparkMax m_arm_up_down_motor = new CANSparkMax(Constants.ARM_UP_DOWN_MOTOR_ID, MotorType.kBrushless);
  public static DigitalInput limit_switch = new DigitalInput(Constants.LOWER_ARM_LIMIT_SWITCH_ID);

  /** Creates a new Conveyor_Motor. */
  public Arm_Up_Down_Motor() {
    //m_arm_up_down_motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Run_Arm_Up_Down_Motor(double up_down_speed) {
    m_arm_up_down_motor.set(up_down_speed);
  }

  public void Run_Arm_Up_Down_Motor_With_Joystick(double up_down_joystick_speed) {
    m_arm_up_down_motor.set(up_down_joystick_speed);
  }
}