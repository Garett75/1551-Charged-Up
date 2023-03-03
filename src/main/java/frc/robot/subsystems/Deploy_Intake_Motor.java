// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deploy_Intake_Motor extends SubsystemBase {
  double deploy_intake_motor_speed = Constants.DEPLOY_INTAKE_MOTOR_SPEED;
  public static CANSparkMax m_deploy_intake_motor = new CANSparkMax(Constants.DEPLOY_INTAKE_MOTOR_ID, MotorType.kBrushless);
  public static RelativeEncoder m_encoder = m_deploy_intake_motor.getEncoder();
  public static DigitalInput limit_switch_out = new DigitalInput(Constants.DEPLOY_INTAKE_LIMIT_SWITCH_ID);
  public static DigitalInput limit_switch_in = new DigitalInput(Constants.RETRACT_INTAKE_LIMIT_SWITCH_ID);


  /** Creates a new Conveyor_Motor. */
  public Deploy_Intake_Motor() {
        m_deploy_intake_motor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Run_Deploy_Intake_Motor(double deploy_intake_motor_speed) {
    m_deploy_intake_motor.set(deploy_intake_motor_speed);
  }
}