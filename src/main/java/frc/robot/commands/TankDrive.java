// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  private final DriveTrain m_drivetrain;
  PS4Controller ps4controller1 = new PS4Controller(Constants.PS4CONTROLLER_DRIVE_ID);
  double state = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(DoubleSupplier leftspeed, DoubleSupplier rightspeed, DriveTrain driveTrain) {
    m_drivetrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if (state == 0) {
      double leftspeed = ps4controller1.getRawAxis(1);
      double rightspeed = ps4controller1.getRawAxis(5);
      
      leftspeed = (leftspeed <= 0.0 ? 1.0 : -1.0) * leftspeed * leftspeed;

      rightspeed = (rightspeed <= 0.0 ? 1.0 : -1.0) * rightspeed * rightspeed;

      m_drivetrain.drive(leftspeed, rightspeed);
    }

    if (state == 1) {
      double leftspeed = ps4controller1.getRawAxis(1);
      double rightspeed = ps4controller1.getRawAxis(5);
      
      leftspeed = (leftspeed <= 0.0 ? 1.0 : -1.0) * leftspeed * leftspeed;

      rightspeed = (rightspeed <= 0.0 ? 1.0 : -1.0) * rightspeed * rightspeed;
      
      m_drivetrain.drive(-rightspeed, -leftspeed);
    }

    if (state == 1) {
      if (ps4controller1.getR1ButtonPressed()) {
        state = 0;
      }
    }

    if (state == 0) {
       if (ps4controller1.getR1ButtonPressed()) {
        state = 1;
       }
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
    return false;
  }
}
