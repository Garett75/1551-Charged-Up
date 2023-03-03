// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor_Motor;
import frc.robot.subsystems.Intake_Motor;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Spit_Out_Command_Group extends SequentialCommandGroup {
  /** Creates a new Intake_Out_Command_Group. */
  public Spit_Out_Command_Group(Pneumatics pneumatics, Intake_Motor m_intake_motor, Conveyor_Motor conveyor_motor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
    new Intake_Out(pneumatics),
    new ParallelCommandGroup(
      new Feed_Out(m_intake_motor),
      new Conveyor_Reverse(conveyor_motor)
      )
    );
  }
}
