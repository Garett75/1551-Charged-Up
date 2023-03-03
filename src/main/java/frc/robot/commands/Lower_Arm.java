// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm_Up_Down_Motor;
import frc.robot.subsystems.Extend_Arm_Motor;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class Lower_Arm extends CommandBase {
  private Pneumatics pneumatics;
  public static boolean stop;
  private Arm_Up_Down_Motor m_arm_up_down_motor;
  /** Creates a new Raise_Arm_High. */
  public Lower_Arm(Arm_Up_Down_Motor m_arm_up_down_motor, Pneumatics pneumatics) {
    this.m_arm_up_down_motor = m_arm_up_down_motor;
    this.pneumatics = pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((m_arm_up_down_motor));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    stop = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Arm_Up_Down_Motor.limit_switch.get()) {
      m_arm_up_down_motor.Run_Arm_Up_Down_Motor(0);
      stop = true;
    }
    else {
      m_arm_up_down_motor.Run_Arm_Up_Down_Motor(-Constants.ARM_UP_DOWN_SPEED);

    }
  
    SmartDashboard.putBoolean("stop", stop);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Extend_Arm_Motor.m_encoder.setPosition(0);
    m_arm_up_down_motor.Run_Arm_Up_Down_Motor(0);
    pneumatics.arm_extend_brake_solenoid.set(DoubleSolenoid.Value.kReverse);
    
    Robot.m_encoder.reset();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
