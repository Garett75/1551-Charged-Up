// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.ADIS16470_IMU.IMUAxis;

public class DriveTrain extends SubsystemBase {
  public DifferentialDriveKinematics kinematics;
  public DifferentialDrivePoseEstimator drivePoseEstimator;
  public static double xaxis;
  public static double yaxis;
  public static double zaxis;
  public static ADIS16470_IMU imu = new ADIS16470_IMU();
  public static final CANSparkMax m_right_1 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_ID_1, MotorType.kBrushless);
  public static final CANSparkMax m_left_1 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_ID_1, MotorType.kBrushless);
  public static final CANSparkMax m_right_2 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_ID_2, MotorType.kBrushless);
  public static final CANSparkMax m_left_2 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_ID_2, MotorType.kBrushless);
  public static RelativeEncoder right_encoder = m_right_1.getEncoder();
  public static RelativeEncoder left_encoder = m_left_1.getEncoder();
  public static double right_current = m_right_1.getOutputCurrent();
  public static double left_current = m_left_1.getOutputCurrent();


  public static PIDController leftPIDController = new PIDController(100.44, 0, 11.715);
  public static PIDController rightPIDController = new PIDController(100.44, 0, 11.715);
  


  private final MotorController m_leftMotorGroup = new MotorControllerGroup
    (m_left_1, m_left_2);
    public final static MotorController m_rightMotorGroup = new MotorControllerGroup
    (m_right_1, m_right_2);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    imu.calibrate();
    
    m_left_1.setInverted(false);
    m_left_2.setInverted(false);
    m_right_1.setInverted(true);
    m_right_2.setInverted(true);
    m_left_1.getEncoder().setPositionConversionFactor(.477/10.75);
    m_right_1.getEncoder().setPositionConversionFactor(.477/10.75);
    m_left_1.getEncoder().setVelocityConversionFactor(.477/10.75/60);
    m_right_1.getEncoder().setVelocityConversionFactor(.477/10.75/60);

    
    kinematics = new DifferentialDriveKinematics(.556);
    drivePoseEstimator = new DifferentialDrivePoseEstimator(
                                                            kinematics, 
                                                            Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kX)), 
                                                            m_left_1.getEncoder().getPosition(), 
                                                            m_right_1.getEncoder().getPosition(), 
                                                            new Pose2d());
    m_right_1.burnFlash();
    m_right_2.burnFlash();
    m_left_1.burnFlash();
    m_left_2.burnFlash();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    m_left_1.getOutputCurrent();
    
    xaxis = imu.getAngle(IMUAxis.kX);
    yaxis = imu.getAngle(IMUAxis.kY);
    zaxis = imu.getAngle(IMUAxis.kZ);
    SmartDashboard.putNumber("x-axis", xaxis);
    SmartDashboard.putNumber("y-axis", yaxis);
    SmartDashboard.putNumber("z-axis", zaxis);

    drivePoseEstimator.update(
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kX)),  
      m_left_1.getEncoder().getPosition(), 
      m_right_1.getEncoder().getPosition());
    
    LimelightHelpers.getBotPose2d_wpiBlue(getName());
    LimelightHelpers.Results llResults =
        LimelightHelpers.getLatestResults("limelight").targetingResults;
    if(llResults.valid && llResults.botpose[0] != 0.0 && llResults.botpose[1] != 0.0){
      Pose2d visionPose = new Pose2d();
      if( DriverStation.getAlliance() == Alliance.Blue){
        visionPose = llResults.getBotPose2d_wpiBlue();
      }else{
        visionPose = llResults.getBotPose2d_wpiRed();
      }
      drivePoseEstimator.addVisionMeasurement(visionPose,
        Timer.getFPGATimestamp()-((llResults.latency_capture + llResults.latency_pipeline + llResults.latency_jsonParse) / 1000.0));
    }
    
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_left_1.getEncoder().getVelocity(), m_right_1.getEncoder().getVelocity());
  }

  public Pose2d getPose(){
    return drivePoseEstimator.getEstimatedPosition();
  }
  
  public void setPose(Pose2d newPose){
    drivePoseEstimator.resetPosition(
              Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kX)), 
              m_left_1.getEncoder().getPosition(), 
              m_right_1.getEncoder().getPosition(), 
              newPose);
  }

  public void drive(double leftspeed, double rightspeed) {
    m_drive.tankDrive(leftspeed,rightspeed);
  }

  public void driveInVolts(double leftVolts, double rightVolts){
    drive(leftVolts/12, rightVolts/12);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
