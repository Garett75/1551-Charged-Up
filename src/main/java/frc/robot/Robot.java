// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto_Drive;
import frc.robot.subsystems.Arm_Up_Down_Motor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Extend_Arm_Motor;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  PowerDistribution power_hub = new PowerDistribution(1, ModuleType.kRev);
  double left_drive_1_current = power_hub.getCurrent(5);
  double conveyor_motor_current = power_hub.getCurrent(7);

  /*public static final int LEFT_DRIVE_MOTOR_ID_2 = 4;
public static final int LEFT_DRIVE_MOTOR_ID_1 = 5;
public static final int RIGHT_DRIVE_MOTOR_ID_2 = 10;
public static final int RIGHT_DRIVE_MOTOR_ID_1 = 11;
public static final int CONVEYOR_MOTOR_ID = 7;
public static final int BOTTOM_WINCH_MOTOR_ID = 8;
public static final int INTAKE_MOTOR_ID = 6;
public static final int ARM_UP_DOWN_MOTOR_ID = 1;
public static final int EXTEND_ARM_MOTOR_ID = 2;
public static final int DEPLOY_INTAKE_MOTOR_ID = 0;*/
  
  public static int state = 0;
  public static double confidence = 0;
  public static int t = 0;
  public static Encoder m_encoder;
  public static boolean robot_state = true;
  public static boolean gamePieceReady = false;
  public static boolean gamePieceDetected = false;
  public static double[] proxArray = new double[50];
  public static int counter;
  public static int flash1;
  public static int flash2;
  //public static RelativeEncoder left_encoder = DriveTrain.m_left_1.getEncoder();
 // public static RelativeEncoder right_encoder = DriveTrain.m_right_1.getEncoder();
  //AddressableLED m_led = new AddressableLED(0);
  public static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);  
  public static boolean beam_broken_front;
  public static boolean beam_broken_back;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final DigitalInput conveyor_beam_break = new DigitalInput(4);
  private Timer gamePieceTimer = new Timer();
  final PS4Controller ps4Controller = new PS4Controller(Constants.PS4CONTROLLER_OTHER_ID);
  final JoystickButton left_one2 = new JoystickButton(ps4Controller, 10);
  final JoystickButton right_one2 = new JoystickButton(ps4Controller, 9);
  boolean flash_yellow = left_one2.getAsBoolean();
  boolean flash_purple = right_one2. getAsBoolean();


  private final Color kYellowTarget = new Color(0.335, 0.545, 0.100);
  private final Color kPurpleTarget = new Color(0.220, 0.350, 0.420);

  /*public enum State{
    kNoObject,kCone,kCube
  }
  
  public State robotState = State.kCone; */
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    DriveTrain.left_encoder.setPosition(0);
    DriveTrain.right_encoder.setPosition(0);

    
    m_encoder = new Encoder(0, 1);
    

   // m_led.setLength(m_ledBuffer.getLength());
   // m_led.setData(m_ledBuffer);
    //m_led.start();
    gamePieceTimer.reset();
    gamePieceTimer.stop(); 
  }

   /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    double left_encoder_position = DriveTrain.left_encoder.getPosition();
    double right_encoder_position = DriveTrain.right_encoder.getPosition();
    double extend_encoder = Extend_Arm_Motor.m_encoder.getPosition();

    int proximity = m_colorSensor.getProximity();

    beam_broken_back = conveyor_beam_break.get();


    Arm_Up_Down_Motor.m_arm_up_down_motor.burnFlash();
  
    Color detectedColor = m_colorSensor.getColor();
    

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kYellowTarget){
      colorString = "Yellow";
      state = 1;
    } else if (match.color == kPurpleTarget) {
      colorString = "Purple";
      state = 2;
    } else {
      colorString = "Unknown";
      state = 0;
    }

      if (proximity < 130) {
        gamePieceDetected = false;
        gamePieceReady = false;
      }

    
    if (Robot.state == 1 && Robot.confidence > .89) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
         m_ledBuffer.setRGB(i, 255, 233, 0);
      }
  
      if (beam_broken_back) {
        gamePieceReady = true;
      }

      if (!gamePieceReady && proximity > 135) {
        gamePieceDetected = true;
      }

      if (gamePieceReady && proximity > 135) {
        gamePieceTimer.start();
      }
      if (gamePieceTimer.hasElapsed(.25)) {
        gamePieceDetected = true;
        gamePieceTimer.stop();
        gamePieceTimer.reset();
        beam_broken_back = false;
      }
    }
  
    if (Robot.state == 2) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 128);
      }
      if (proximity > 140) {
        gamePieceDetected = true;
      }
      beam_broken_back = false;
    }


    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kPurpleTarget);



    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);


    SmartDashboard.putNumber("Arm Position", m_encoder.getDistance());
    SmartDashboard.putBoolean("gamepiece detected", gamePieceDetected);
    SmartDashboard.putNumber("proximity", proximity);
    SmartDashboard.putBoolean("conveyor_beam_break", beam_broken_back);
    SmartDashboard.putBoolean("gamepiece ready", gamePieceReady);
    SmartDashboard.putNumber("left encoder", left_encoder_position);
    SmartDashboard.putNumber("right encoder", -right_encoder_position);
    SmartDashboard.putNumber("extend encoder", extend_encoder);
    SmartDashboard.putNumber("left current 1", left_drive_1_current);
    SmartDashboard.putNumber("conveyor current", conveyor_motor_current);


    if (flash_yellow == true && flash1 == 1) {
      for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      Robot.m_ledBuffer.setRGB(i, 195, 0, 67);
      flash1 = 0;
      }
    }

    if (flash_yellow == true && flash1 == 0) {
        for (var i = 1; i < Robot.m_ledBuffer.getLength(); i++) {
          Robot.m_ledBuffer.setRGB(i, 0, 0, 0);
          flash1 = 1;
        }
    }

    if (flash_purple == true && flash2 == 1) {
      for (var c = 0; c < Robot.m_ledBuffer.getLength(); c++) {
      Robot.m_ledBuffer.setRGB(c, 255, 233, 0);
      flash2 = 0;
      } 
    }

    if (flash_purple == true && flash2 == 0) {
        for (var i = 1; i < Robot.m_ledBuffer.getLength(); i++) {
          Robot.m_ledBuffer.setRGB(i, 0, 0, 0);
          flash2 = 1;
        }
    }

    confidence = match.confidence;
  }

    /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
