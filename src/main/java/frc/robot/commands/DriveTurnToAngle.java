// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTurnToAngle extends CommandBase {
  DriveTrain driveTrain;
  DoubleSupplier angleDeg;
  double targetAngle;
  PIDController pidController = new PIDController(.05, 0, 0);
  int onTarget;
  /** Creates a new DriveTurnToAngle. */
  public DriveTurnToAngle(DriveTrain driveTrain, DoubleSupplier angleDeg) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.angleDeg = angleDeg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = angleDeg.getAsDouble() + driveTrain.yaxis;
    pidController.reset();
    onTarget = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(driveTrain.yaxis,targetAngle);
    driveTrain.drive(-output, output);
    if(Math.abs(driveTrain.yaxis-targetAngle)<5){
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
    return onTarget>=10;
  }
}
