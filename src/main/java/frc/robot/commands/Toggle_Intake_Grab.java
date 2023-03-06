// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class Toggle_Intake_Grab extends CommandBase {
  public final Pneumatics pneumatics;
  /** Creates a new Intake_In. */
  public Toggle_Intake_Grab(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneumatics);
    this.pneumatics = pneumatics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    if (pneumatics.intake_grab_solenoid.get() == Value.kForward) {
    pneumatics.intake_grab_solenoid.set(DoubleSolenoid.Value.kReverse);
    Pneumatics.intake_grab_state = false;
  }

  else {
    pneumatics.intake_grab_solenoid.set(DoubleSolenoid.Value.kForward);
    Pneumatics.intake_grab_state = true;
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
