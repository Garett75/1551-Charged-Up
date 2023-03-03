// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extend_Arm_Motor;

public class Retract_Arm extends CommandBase {
  private Extend_Arm_Motor extend_arm_motor;
  private boolean stop;

  /** Creates a new Extend_Arm. */
  public Retract_Arm(Extend_Arm_Motor extend_arm_motor) {
        // Use addRequirements() here to declare subsystem dependencies.
    this.extend_arm_motor = extend_arm_motor;
    addRequirements(extend_arm_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Extend_Arm_Motor.m_extend_arm_motor.restoreFactoryDefaults();
    Extend_Arm_Motor.m_extend_arm_motor.burnFlash();
    stop = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (!extend_arm_motor.limit_switch.get()) {
        stop = true;
      }
      else {
  extend_arm_motor.Run_Extend_Arm_Motor(Constants.RETRACT_ARM_SPEED);
      }
  
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extend_arm_motor.Run_Extend_Arm_Motor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
