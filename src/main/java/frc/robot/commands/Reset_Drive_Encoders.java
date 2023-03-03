// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Reset_Drive_Encoders extends CommandBase {
  public final DriveTrain m_drivetrain;
  public static RelativeEncoder m_encoder;
  public static RelativeEncoder m_encoder2;


  /** Creates a new Auto_Drive. */
  public Reset_Drive_Encoders(DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
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
    DriveTrain.right_encoder.setPosition(0);
    DriveTrain.left_encoder.setPosition(0);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
