// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sequence.ArmHomeSequence;
import frc.robot.sequence.DunkSequence;
import frc.robot.sequence.FloorSequence;
import frc.robot.sequence.MidDunkSequence;
import frc.robot.sequence.Timer;
import frc.robot.sequence.swerve.CubeSequence;
import frc.robot.sequence.swerve.LeftConeSequence;
import frc.robot.sequence.swerve.RightConeSequence;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Robot obj = null;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private XboxController controller;
  private XboxController controller2;

  private Swerve swerveDriveTrain = new Swerve();

  private Arm arm = new Arm();
  private Ramp ramp = new Ramp();

  public DunkSequence dunkSequence = new DunkSequence(arm, ramp);
  public ArmHomeSequence homeSequence = new ArmHomeSequence(arm, ramp);
  public MidDunkSequence midDunkSequence = new MidDunkSequence(arm, ramp);
  public FloorSequence floorSequence = new FloorSequence(arm, ramp);
  public RightConeSequence rightConeSequence = new RightConeSequence(swerveDriveTrain);
  public CubeSequence cubeSequence = new CubeSequence(swerveDriveTrain);
  public LeftConeSequence leftConeSequence = new LeftConeSequence(swerveDriveTrain);

  private int lastAprilID;

  private UsbCamera camera1 = null;
  private UsbCamera camera2 = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    super();
    Robot.obj = this;
  }

  public boolean areSequencesComplete() {
    return dunkSequence.isComplete() && homeSequence.isComplete() && midDunkSequence.isComplete() && leftConeSequence.isComplete() && cubeSequence.isComplete() && rightConeSequence.isComplete();
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    controller = new XboxController(0);
    controller2 = new XboxController(1);
    
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    // sharedInit(); why was this here?? questions for next time

    // swerveDriveTrain.robotInit();

    
  }

  //Calibrates motors so that their relative encoders agree with absolute encoders
  private void calibrateEncoders() {
    swerveDriveTrain.calibrateEncoders();
  }

  private void sharedInit() {
    calibrateEncoders();
    swerveDriveTrain.sharedInit();
    arm.sharedInit();
    ramp.sharedInit();
    Timer.clear();
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
    // grabberMotor.config_kP(0, 0.4);
    // grabberMotor.config_kI(0, 0);
    // grabberMotor.config_kD(0, 0);
    // grabberMotor.config_kF(0, 0);

    // wristPIDController.setVariables(
    //   0.005,
    //   0.0000005,
    //   0,
    //   0,
    //   0,
    //   1.0,
    //   -1.0);


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // beltController.setAngle(360);
  }

  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if(this.areSequencesComplete()) {
      if(swerveDriveTrain.aprilID != -1 && swerveDriveTrain.aprilID != 4 && swerveDriveTrain.aprilID != 5) {
        this.lastAprilID = swerveDriveTrain.aprilID;
      }

      boolean cubeMode = controller2.getLeftBumper();

      if (cubeMode) {

      } else {
        if(controller2.getYButtonPressed()) {
          dunkSequence.run();
        } else if(controller2.getXButton()) {
          midDunkSequence.run();
        } else if (controller2.getBButton()) {
          homeSequence.run();
        } else if (controller2.getAButton()) {
          floorSequence.run();
        } else if(controller.getXButton()) {
          leftConeSequence.setTargetID(lastAprilID);
          leftConeSequence.run();
        } else if(controller.getAButton()) {
          cubeSequence.setTargetID(lastAprilID);
          cubeSequence.run();
        } else if(controller.getBButton()) {
          rightConeSequence.setTargetID(lastAprilID);
          rightConeSequence.run();
        }
      }
      
    }

    swerveDriveTrain.update(controller);
    arm.update(controller, controller2);
    ramp.update(controller, controller2);

    swerveDriveTrain.smartDashboard();
    ramp.smartDashboard();

    Timer.checkAllTimers(controller);
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
