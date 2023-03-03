// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
//Controller ID Values
public static final int PS4CONTROLLER_DRIVE_ID = 0;
public static final int PS4CONTROLLER_OTHER_ID = 1;

//Motor ID Values
public static final int LEFT_DRIVE_MOTOR_ID_2 = 4;
public static final int LEFT_DRIVE_MOTOR_ID_1 = 5;
public static final int RIGHT_DRIVE_MOTOR_ID_2 = 10;
public static final int RIGHT_DRIVE_MOTOR_ID_1 = 11;
public static final int CONVEYOR_MOTOR_ID = 7;
public static final int BOTTOM_WINCH_MOTOR_ID = 8;
public static final int INTAKE_MOTOR_ID = 6;
public static final int ARM_UP_DOWN_MOTOR_ID = 1;
public static final int EXTEND_ARM_MOTOR_ID = 2;
public static final int DEPLOY_INTAKE_MOTOR_ID = 0;


//Speed Values
public static final double CONVEYOR_SPEED = .75;
public static final double BOTTOM_WINCH_SPEED = .5;
public static final double INTAKE_CUBE_SPEED = .7;
public static final double INTAKE_CONE_SPEED = 1;
public static final double ARM_UP_DOWN_SPEED = .6;
public static final double EXTEND_ARM_SPEED = .4;
public static final double RETRACT_ARM_SPEED = -.4;
public static final double DRIVE_SPEED = .3;
public static final double LOWERING_RETRACT_ARM_SPEED = -.5;
public static final double DEPLOY_INTAKE_MOTOR_SPEED = .5;
public static final double RETRACT_INTAKE_MOTOR_SPEED = .5;


//Solenoid ID Values
public static final int BOTTOM_PLATE_UP = 13;
public static final int BOTTOM_PLATE_DOWN = 3;
public static final int ARM_EXTEND_BRAKE_ENGAGE = 6;
public static final int ARM_EXTEND_BRAKE_DISENGAGE = 10;
public static final int ARM_RAISE_LOWER_BRAKE_DISENGAGE = 4;
public static final int ARM_RAISE_LOWER_BRAKE_ENGAGE = 8;
public static final int CLAW_OPEN = 11;
public static final int CLAW_CLOSE = 1;
public static final int INTAKE_SOLENOID_IN = 9;
public static final int INTAKE_SOLENOID_OUT = 2;

//RoboRio Input Values
public static final int RETRACT_LIMIT_SWITCH_ID = 2;
public static final int LOWER_ARM_LIMIT_SWITCH_ID = 3;
public static final int DEPLOY_INTAKE_LIMIT_SWITCH_ID = 0;
public static final int RETRACT_INTAKE_LIMIT_SWITCH_ID = 0;

//Distance Values
public static final double EXTEND_ROTATIONS_HIGH = 85;
public static final int LED_LENGTH = 126;
public static final double RAISE_ARM_HIGH_TARGET = 420; 
public static final int RAISE_ARM_MEDIUM_TARGET = 360;
public static final int RAISE_ARM_LOW_TARGET = 115;
public static final double AUTO_DISTANCE_1 = 1;
public static final double EXTEND_ROTATIONS_LOW = 20;


}
