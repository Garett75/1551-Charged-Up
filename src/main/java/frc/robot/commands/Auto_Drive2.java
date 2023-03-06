// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Auto_Drive2 extends CommandBase {
  public final DriveTrain m_drivetrain;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoder2;
  Timer timer = new Timer();
  boolean drive_stop = false;


  /** Creates a new Auto_Drive. */
  public Auto_Drive2(double driveSpeed, double driveSpeed2, DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.m_right_1.restoreFactoryDefaults();
    DriveTrain.m_right_1.burnFlash();
    m_encoder = DriveTrain.m_right_1.getEncoder();

    DriveTrain.m_left_1.restoreFactoryDefaults();
    DriveTrain.m_left_1.burnFlash();
    m_encoder2 = DriveTrain.m_left_1.getEncoder();

    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftDistance = m_encoder.getPosition();
    double rightDistance = m_encoder2.getPosition();
    double error = leftDistance - rightDistance;
    double leftspeed = Constants.DRIVE_SPEED - error;
    double rightspeed = Constants.DRIVE_SPEED + error;

    m_drivetrain.drive(leftspeed, rightspeed);

    if(timer.hasElapsed(3)) {
      drive_stop = true;
    }
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive_stop;
  }
}
