// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake_Motor;

public class Feed_Cube_In extends CommandBase {
  private Intake_Motor m_intake_motor;

  /** Creates a new Feed_In. */
  public Feed_Cube_In(Intake_Motor m_intake_motor) {
    this.m_intake_motor = m_intake_motor;
    addRequirements(m_intake_motor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake_motor.Run_Intake_Motor_Cube(-Constants.INTAKE_CUBE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake_motor.Run_Intake_Motor_Cube(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
