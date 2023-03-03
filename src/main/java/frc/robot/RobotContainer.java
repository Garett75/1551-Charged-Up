// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto_Drive;
import frc.robot.commands.Auto_Drive2;
import frc.robot.commands.Bottom_Winch_Unspool;
import frc.robot.commands.Conveyor_Forward_And_Grab_Command_Group;
import frc.robot.commands.Extend_Arm_Manual;
import frc.robot.commands.Feed_Cone_In;
import frc.robot.commands.Feed_Cube_In;
import frc.robot.commands.Intake_In;
import frc.robot.commands.Intake_Out;
import frc.robot.commands.Lower_Arm;
import frc.robot.commands.Lower_Arm_Command_Group;
import frc.robot.commands.Lower_Arm_Command_Group_Low;
import frc.robot.commands.Lowering_Retract_Arm;
import frc.robot.commands.Raise_Arm_Manual;
import frc.robot.commands.Reset_Drive_Encoders;
import frc.robot.commands.Score_High_Command_Group;
import frc.robot.commands.Score_Low_Command_Group;
import frc.robot.commands.Score_Medium_Command_Group;
import frc.robot.commands.Spit_Out_Command_Group;
import frc.robot.commands.TankDrive;
import frc.robot.commands.Toggle_Claw;
import frc.robot.commands.Toggle_Shenanigan;
import frc.robot.subsystems.Arm_Up_Down_Motor;
import frc.robot.subsystems.Bottom_Winch_Motor;
import frc.robot.subsystems.Conveyor_Motor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake_Motor;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Extend_Arm_Motor;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Initialize PS4Controllers
  PS4Controller ps4controller1 = new PS4Controller(Constants.PS4CONTROLLER_DRIVE_ID);
  PS4Controller ps4controller2 = new PS4Controller(Constants.PS4CONTROLLER_OTHER_ID);

  //Initianlize Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Arm_Up_Down_Motor arm_up_down_motor = new Arm_Up_Down_Motor();
  private final Bottom_Winch_Motor bottom_winch_motor = new Bottom_Winch_Motor();
  public final Conveyor_Motor conveyor_motor = new Conveyor_Motor();
  private final Intake_Motor intake_motor = new Intake_Motor();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Extend_Arm_Motor extend_arm_motor = new Extend_Arm_Motor();

  //Initialize Commands
  private final Intake_In intake_in = new Intake_In(pneumatics);
  private final Bottom_Winch_Unspool bottom_winch_unspool = new Bottom_Winch_Unspool(bottom_winch_motor);
  private final Toggle_Claw toggle_claw = new Toggle_Claw(pneumatics);
  private final Feed_Cube_In feed_cube_in = new Feed_Cube_In(intake_motor);
  private final Feed_Cone_In feed_cone_in = new Feed_Cone_In(intake_motor);
  private final Intake_Out intake_out = new Intake_Out(pneumatics);
  private final Auto_Drive auto_drive = new Auto_Drive(Constants.DRIVE_SPEED, Constants.DRIVE_SPEED, driveTrain);
  private final Auto_Drive2 auto_drive2 = new Auto_Drive2(Constants.DRIVE_SPEED, Constants.DRIVE_SPEED, driveTrain);
  private final Toggle_Shenanigan toggle_shenanigan = new Toggle_Shenanigan(pneumatics);
  private final Reset_Drive_Encoders reset_drive_encoders = new Reset_Drive_Encoders(driveTrain);
  private final Score_High_Command_Group score_high_command_group = new Score_High_Command_Group(arm_up_down_motor, pneumatics, extend_arm_motor);
  private final Score_Medium_Command_Group score_medium_command_group = new Score_Medium_Command_Group(arm_up_down_motor, pneumatics, extend_arm_motor);
  private final Score_Low_Command_Group score_low_command_group = new Score_Low_Command_Group(arm_up_down_motor, pneumatics, extend_arm_motor);
  private final Lower_Arm_Command_Group lower_arm_command_group = new Lower_Arm_Command_Group(arm_up_down_motor, pneumatics, extend_arm_motor);
  private final Lower_Arm_Command_Group_Low lower_arm_command_group_low = new Lower_Arm_Command_Group_Low(arm_up_down_motor, pneumatics, extend_arm_motor);
  private final Spit_Out_Command_Group spit_out_command_group = new Spit_Out_Command_Group(pneumatics, intake_motor, conveyor_motor);
  private final Conveyor_Forward_And_Grab_Command_Group conveyor_forward_and_grab_command_group = new Conveyor_Forward_And_Grab_Command_Group(conveyor_motor, pneumatics, extend_arm_motor);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain.setDefaultCommand(
        new TankDrive(() -> -ps4controller1.getRawAxis(1), () -> -ps4controller1.getRawAxis(5), driveTrain));
    arm_up_down_motor.setDefaultCommand(
        new Raise_Arm_Manual(() -> ps4controller2.getRawAxis(1), arm_up_down_motor));
    extend_arm_motor.setDefaultCommand(
        new Extend_Arm_Manual(() -> ps4controller2.getRawAxis(5), extend_arm_motor));
    
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    final JoystickButton left_one1 = new JoystickButton(ps4controller1, 5);
    left_one1.onTrue(intake_out);
    left_one1.whileTrue(feed_cone_in);
    //left_one1.onTrue(Commands.race(new Conveyor_Forward_And_Grab_Command_Group(conveyor_motor, pneumatics, extend_arm_motor),new WaitCommand(4)));
    left_one1.onFalse(intake_in);
    left_one1.onTrue(conveyor_forward_and_grab_command_group);
    left_one1.onFalse(conveyor_forward_and_grab_command_group);

    final JoystickButton left_paddle1 = new JoystickButton(ps4controller1, 7);
    left_paddle1.whileTrue(spit_out_command_group);
    left_paddle1.onFalse(intake_in);

    final JoystickButton trianglebutton2 = new JoystickButton(ps4controller2, 4);
    trianglebutton2.onTrue(score_high_command_group);

    final JoystickButton circlebutton2 = new JoystickButton(ps4controller2, 3);
    circlebutton2.onTrue(score_medium_command_group);

    final JoystickButton xbutton2 = new JoystickButton(ps4controller2, 2);
    xbutton2.onTrue(score_low_command_group);

    final JoystickButton squarebutton2 = new JoystickButton(ps4controller2, 1);
    squarebutton2.onTrue(new ConditionalCommand(lower_arm_command_group, lower_arm_command_group_low, () -> Robot.robot_state).until(() -> Lower_Arm.stop && Lowering_Retract_Arm.stop2));

    final JoystickButton options1 = new JoystickButton(ps4controller1, 10);
    options1.whileTrue(toggle_shenanigan);

    final JoystickButton share1 = new JoystickButton(ps4controller1, 9);
    share1.whileTrue(bottom_winch_unspool);

    final JoystickButton share2 = new JoystickButton(ps4controller2, 9);
    share2.whileTrue(toggle_claw);

    final JoystickButton leftpaddlebutton2 = new JoystickButton(ps4controller2, 7);
    leftpaddlebutton2.onTrue(conveyor_forward_and_grab_command_group);   

    final JoystickButton trianglebutton1 = new JoystickButton(ps4controller1, 4);
    trianglebutton1.onTrue(reset_drive_encoders);

    final JoystickButton ps4button = new JoystickButton(ps4controller1, 13);
    ps4button.onTrue(new PPRamseteCommand(
      PathPlanner.loadPath("New Path", new PathConstraints(0, 0)),
      driveTrain::getPose,
      new RamseteController(),
      new SimpleMotorFeedforward(.19699,2.8161, 0.83236),
      driveTrain.kinematics,
      driveTrain::getWheelSpeeds,
      driveTrain.leftPIDController,
      driveTrain.rightPIDController,
      driveTrain::driveInVolts,
      true,
      driveTrain
    ));
    

    

    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auto_drive;
    //return null;
  }
}
