// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private XboxController controller;
  private Swerve swerveDriveTrain = new Swerve();
  
  private DigitalInput limitSwitch = new DigitalInput(4);

  // private final TalonFX grabberMotor = new TalonFX(9);

  private double grabberPosition = 0;
  private double grabberPositionRamp = 0;
  private double grabberRampFactor = 100;
  private boolean homeGrabber = false;

  private CvSink cvSink = null;
  private Mat cameraFrame = null;
  private UsbCamera camera = null;
  private CvSource outputStream = null;
  private AprilTagDetector detector = new AprilTagDetector();
  private double distanceFromCenter = 0;

  // private final float targetPosition = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    controller = new XboxController(0);
    sharedInit();

    swerveDriveTrain.robotInit();

    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240);
    

    cvSink = CameraServer.getVideo();
    detector.addFamily("tag16h5");
    cameraFrame = new Mat(320, 240, CvType.CV_8UC3);
    outputStream = CameraServer.putVideo("Rectangle", 320, 240);

    
  }

  //Calibrates motors so that their relative encoders agree with absolute encoders
  private void calibrateEncoders() {
    swerveDriveTrain.calibrateEncoders();
  }

  private void sharedInit() {
    calibrateEncoders();
    swerveDriveTrain.sharedInit();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

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
    
    sharedInit();

    // grabberMotor.setSelectedSensorPosition(grabberPosition);
    // grabberMotor.config_kP(0, 0.1);
    // grabberMotor.config_kI(0, 0);
    // grabberMotor.config_kD(0, 0);
    // grabberMotor.config_kF(0, 0);
    homeGrabber = false;


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // beltController.setAngle(360);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    SmartDashboard.putNumber("tx", limelight.getEntry("tx").getDouble(-9999.314159));
    SmartDashboard.putNumber("ty", limelight.getEntry("ty").getDouble(-9999.314159));

    if(controller.getAButton()) {
      homeGrabber = true;
    }

    SmartDashboard.putBoolean("limit switch", limitSwitch.get());

    if(homeGrabber) {
      grabberRampFactor = 60;
      grabberPosition = 4000;
    }

    if(limitSwitch.get() && homeGrabber) {
      // grabberMotor.setSelectedSensorPosition(400);
      homeGrabber = false;
      grabberPosition = 0;
      grabberPositionRamp = 400;
      grabberRampFactor = 100;
    }

    if(!homeGrabber) {
      if(controller.getXButton()) {
        grabberPosition = -4500;
      } else {
        grabberPosition = 0;
      }
    }

    if(grabberPositionRamp + grabberRampFactor * 5 < grabberPosition) {
      grabberPositionRamp += grabberRampFactor;
    } else if(grabberPositionRamp - grabberRampFactor * 5 > grabberPosition) {
      grabberPositionRamp -= grabberRampFactor;
    }

    swerveDriveTrain.update(controller);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
