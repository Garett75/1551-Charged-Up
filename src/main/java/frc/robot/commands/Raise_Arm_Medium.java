// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm_Up_Down_Motor;

public class Raise_Arm_Medium extends CommandBase {
  private Arm_Up_Down_Motor m_arm_up_down_motor;
  public static double goal2 = -Constants.RAISE_ARM_MEDIUM_TARGET;
  boolean stop;
  /** Creates a new Raise_Arm_High. */
  public Raise_Arm_Medium(Arm_Up_Down_Motor m_arm_up_down_motor) {
    this.m_arm_up_down_motor = m_arm_up_down_motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((m_arm_up_down_motor));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop = false;
    Robot.robot_state = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    goal2 = (Constants.RAISE_ARM_MEDIUM_TARGET -Math.abs(Robot.m_encoder.getDistance()));

    if (goal2 > 0) {
      m_arm_up_down_motor.Run_Arm_Up_Down_Motor(Constants.ARM_UP_DOWN_SPEED);
    }

    else {
      m_arm_up_down_motor.Run_Arm_Up_Down_Motor(0);
      stop = true;
    }    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm_up_down_motor.Run_Arm_Up_Down_Motor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
