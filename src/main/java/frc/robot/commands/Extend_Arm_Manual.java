// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extend_Arm_Motor;

public class Extend_Arm_Manual extends CommandBase {
  private final Extend_Arm_Motor m_extend_arm_motor;
  PS4Controller ps4controller2 = new PS4Controller(Constants.PS4CONTROLLER_OTHER_ID);

  
  /** Creates a new Extend_Arm. */
  public Extend_Arm_Manual(DoubleSupplier extend_joystick_speed, Extend_Arm_Motor extend_arm_motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_extend_arm_motor = extend_arm_motor;
    addRequirements(m_extend_arm_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double extend_joystick_speed= ps4controller2.getRawAxis(5);
    m_extend_arm_motor.Run_Arm_Extend_Motor_With_Joystick(-extend_joystick_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extend_arm_motor.Run_Extend_Arm_Motor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
