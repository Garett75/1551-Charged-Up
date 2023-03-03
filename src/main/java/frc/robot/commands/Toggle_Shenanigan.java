// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Toggle_Shenanigan extends CommandBase {
  public final Pneumatics pneumatics;
  /** Creates a new Intake_In. */
  public Toggle_Shenanigan(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneumatics);
    this.pneumatics = pneumatics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (pneumatics.bottom_plate_solenoid.get() == Value.kForward) {
      pneumatics.bottom_plate_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    else {
      pneumatics.bottom_plate_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //pneumatics.claw_solenoid.toggle();
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
