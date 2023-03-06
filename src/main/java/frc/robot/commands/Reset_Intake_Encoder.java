// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Deploy_Intake_Motor;

public class Reset_Intake_Encoder extends CommandBase {
  public final Deploy_Intake_Motor m_deploy_intake_motor;


  /** Creates a new Auto_Drive. */
  public Reset_Intake_Encoder(Deploy_Intake_Motor m_deploy_intake_motor) {
    this.m_deploy_intake_motor = m_deploy_intake_motor;
    addRequirements(m_deploy_intake_motor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    for (var i = 0; i < 5; i++) {
    Deploy_Intake_Motor.intake_encoder.setPosition(0);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
