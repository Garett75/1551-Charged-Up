// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Bottom_Winch_Motor;

public class Bottom_Winch_Pull extends CommandBase {
  private Bottom_Winch_Motor bottom_winch_motor;
  /** Creates a new Conveyor_Forward. */
  public Bottom_Winch_Pull(Bottom_Winch_Motor bottom_winch_motor) {
    this.bottom_winch_motor = bottom_winch_motor;
    addRequirements(bottom_winch_motor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      bottom_winch_motor.Run_Bottom_Winch_Motor(Constants.BOTTOM_WINCH_SPEED);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bottom_winch_motor.Run_Bottom_Winch_Motor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
