// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  public static PneumaticHub pneumatichub = new PneumaticHub();
  public static boolean intake_grab_state = false;
  public final DoubleSolenoid bottom_plate_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.BOTTOM_PLATE_UP, Constants.BOTTOM_PLATE_DOWN);
  public final DoubleSolenoid arm_extend_brake_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.ARM_EXTEND_BRAKE_ENGAGE, Constants.ARM_EXTEND_BRAKE_DISENGAGE);
  public final DoubleSolenoid arm_raise_lower_brake_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.ARM_RAISE_LOWER_BRAKE_DISENGAGE, Constants.ARM_RAISE_LOWER_BRAKE_ENGAGE);
  public final DoubleSolenoid claw_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.CLAW_OPEN, Constants.CLAW_CLOSE);
  public final DoubleSolenoid intake_grab_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_IN, Constants.INTAKE_SOLENOID_OUT);


  public Pneumatics() {
   pneumatichub.enableCompressorAnalog(100, 120);
  }

  public void Solenoid() {

  }

  @Override 
  public void periodic() {

  }

}
