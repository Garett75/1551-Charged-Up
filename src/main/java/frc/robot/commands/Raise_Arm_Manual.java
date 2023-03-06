// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm_Up_Down_Motor;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Raise_Arm_Manual extends CommandBase {
  private final Arm_Up_Down_Motor m_arm_up_down_motor;
  PS4Controller ps4controller2 = new PS4Controller(Constants.PS4CONTROLLER_OTHER_ID);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Raise_Arm_Manual(DoubleSupplier up_down_joystick_speed, Arm_Up_Down_Motor arm_up_down_motor) {
     m_arm_up_down_motor= arm_up_down_motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm_up_down_motor);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double up_down_joystick_speed = ps4controller2.getRawAxis(1);
    m_arm_up_down_motor.Run_Arm_Up_Down_Motor_With_Joystick(-(up_down_joystick_speed/2));
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm_up_down_motor.Run_Arm_Up_Down_Motor_With_Joystick(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
