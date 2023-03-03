// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake_Motor;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake_Cone_Command_Group extends SequentialCommandGroup {
  /** Creates a new Intake_Out_Command_Group. */
  public Intake_Cone_Command_Group(Pneumatics pneumatics, Intake_Motor m_intake_motor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
    new Intake_Out(pneumatics),
    new Feed_Cone_In(m_intake_motor)//Conveyor Forward is called by the button, not in this command group
    );
  }
}
