// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class Bottom_Plate_Down extends CommandBase {
  public final Pneumatics pneumatics;
  /** Creates a new Intake_In. */
  public Bottom_Plate_Down(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneumatics);
    this.pneumatics = pneumatics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics.bottom_plate_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
