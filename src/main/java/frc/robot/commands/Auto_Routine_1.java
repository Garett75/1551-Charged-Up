// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm_Up_Down_Motor;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Routine_1 extends SequentialCommandGroup {
  /** Creates a new Auto_Routine_1. */
  public Auto_Routine_1(DriveTrain drivetrain, Arm_Up_Down_Motor arm_up_down_motor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
super( /*
  new Reset_Drive_Encoders(drivetrain),
  new ParallelCommandGroup(
    new Raise_Arm_High(arm_up_down_motor),
    new Auto_Drive(Constants.DRIVE_SPEED, Constants.DRIVE_SPEED, drivetrain),
  



 */ );  }
}
