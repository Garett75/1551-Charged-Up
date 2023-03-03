// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Deploy_Intake_Motor;;


public class Retract_Intake extends CommandBase {
  public static boolean stop;
  private Deploy_Intake_Motor m_deploy_intake_Motor;

  /** Creates a new Raise_Arm_High. */
  public Retract_Intake(Deploy_Intake_Motor m_deploy_intake_Motor) {
    this.m_deploy_intake_Motor = m_deploy_intake_Motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((m_deploy_intake_Motor));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    stop = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Deploy_Intake_Motor.limit_switch_in.get()) {
      stop = true;
    }
    else {
      m_deploy_intake_Motor.Run_Deploy_Intake_Motor(Constants.RETRACT_INTAKE_MOTOR_SPEED);

    }    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Deploy_Intake_Motor.m_encoder.setPosition(0);
    m_deploy_intake_Motor.Run_Deploy_Intake_Motor(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
