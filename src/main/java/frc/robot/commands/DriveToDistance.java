// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {  
  DriveTrain driveTrain;
  DoubleSupplier distanceSupp;
  double targetDistance;
  int onTarget;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveTrain driveTrain, DoubleSupplier distanceSupp) {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveTrain);
      this.driveTrain = driveTrain;
      this.distanceSupp = distanceSupp;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDistance = distanceSupp.getAsDouble();
    driveTrain.m_right_1.getEncoder().setPosition(0.0);
    driveTrain.m_left_1.getEncoder().setPosition(0.0);
    onTarget = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightOutput = driveTrain.rightPIDController.calculate(driveTrain.m_right_1.getEncoder().getPosition(),targetDistance);
    double leftOutput = driveTrain.leftPIDController.calculate(driveTrain.m_left_1.getEncoder().getPosition(),targetDistance);
    driveTrain.drive(leftOutput, rightOutput);
    if(Math.abs(driveTrain.m_right_1.getEncoder().getPosition()-targetDistance)<.05 ||
      Math.abs(driveTrain.m_left_1.getEncoder().getPosition()-targetDistance)<.05 ){
      onTarget++;
    }else{
      onTarget=0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget >= 10;
  }
}
