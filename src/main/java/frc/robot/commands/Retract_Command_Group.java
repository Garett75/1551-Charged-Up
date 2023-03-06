// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm_Up_Down_Motor;
import frc.robot.subsystems.Extend_Arm_Motor;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Retract_Command_Group extends SequentialCommandGroup {
  /** Creates a new Shoot_High_Command_Group2. */
  public Retract_Command_Group(Arm_Up_Down_Motor arm_up_down_motor, Pneumatics pneumatics, Extend_Arm_Motor extend_arm_motor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new SequentialCommandGroup(
        new Retract_Arm(extend_arm_motor),
        new Arm_Extend_Brake_Engage(pneumatics)
        )
    );
  }
}
