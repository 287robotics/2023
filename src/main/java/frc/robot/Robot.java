// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
  
  private DigitalInput limitSwitch = new DigitalInput(6); //pressed = false
  private boolean jLimit = true;

  // private final TalonFX grabberMotor = new TalonFX(9);
  private CANSparkMax wristMotor = new CANSparkMax(10, MotorType.kBrushless);
  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(7);

  private CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
  private SparkMaxPIDController_ armPID = new SparkMaxPIDController_(armMotor, 125.709);
  private CANSparkMax armMotor2 = new CANSparkMax(12, MotorType.kBrushless);
  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(9);

  private double armPosition = 0;
  private double armTarget = 0;

  private final SparkMaxPIDController_ wristPID = new SparkMaxPIDController_(wristMotor); //needs counts
  private double wristPosition = 0;
  private double wristTarget = 0;

  private double grabberPosition = 0;
  private double previousGrabberPosition = 0;
  private double grabberPositionRamp = 0;
  private double grabberRampFactor = 100;
  private boolean homeGrabber = false;
  private boolean opened = false;
  private int grabberDirection = 1;

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

    
  }

  //Calibrates motors so that their relative encoders agree with absolute encoders
  private void calibrateEncoders() {
    swerveDriveTrain.calibrateEncoders();
  }

  private void sharedInit() {
    calibrateEncoders();
    swerveDriveTrain.sharedInit();
    armMotor2.follow(armMotor, true);
    armPID.disableMotor();
    // wristMotor.getEncoder().setPosition(0);
    // grabberMotor.set(TalonFXControlMode.Position, -10000);
    // grabberMotor.setSelectedSensorPosition(-10000);
    // armPID.disableMotor();
    grabberPosition = -10000;
    double e = armEncoder.get();

    while (e > 1.0) {
      e -= 1.0;
    }
    
    while (e < 0.0) {
      e += 1.0;
    }

    double ew = wristEncoder.get();

    while (ew > 1.0) {
      ew -= 1.0;
    }
    
    while (ew < 0.0) {
      ew += 1.0;
    }

    // armPosition = e;
    // armTarget = e;
    armPosition = e;
    armTarget = e;
    armPID.setReferencePosition(e);
    armPID.setPosition(e);

    wristPosition = ew;
    wristTarget = ew;
    wristPID.setReferencePosition(ew);
    wristPID.setPosition(ew);
    
    opened = false;
    // double encoderPosition = wristEncoder.get();

    // while (encoderPosition > 1.0) {
    //   encoderPosition -= 1.0;
    // }

    // while (encoderPosition < 0) {
    //   encoderPosition += 1.0;
    // }

    // if (encoderPosition < .5) {
    //   encoderPosition += 1.0;
    // }

    // wristMotor.getEncoder().setPosition(encoderPosition * 100);
    armPID.enableMotor();
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

    homeGrabber = false;


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // beltController.setAngle(360);
  }

  private double[] defaults = {0, 0, 0, 0, 0, 0};

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    Set<String> strings = limelight.getKeys();
    
    for(String s : strings) {
      SmartDashboard.putString(s, "yes");
    }

    double[] pose = limelight.getEntry("botpose").getDoubleArray(defaults);
    double tx = pose[0];
    double ty = pose[1];
    double tz = pose[2];
    double rx = pose[3];
    double ry = pose[4];
    double rz = pose[5];

    
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("april rotation", rz);
    // SmartDashboard.putNumber("wrist", wristEncoder.get());

    // if(controller.getAButton()) {
    //   homeGrabber = true;
    // }

    // if(homeGrabber) {
    //   grabberRampFactor = 60;
    //   grabberPosition = 4000;
    // }

    // if(controller.getXButton()) {
    //   armPID.enableMotor();
    // } else {
    //   armPID.disableMotor();
    // }

    double b = controller.getLeftTriggerAxis();
    double c = controller.getRightTriggerAxis();
    double e = armEncoder.get();

    while(e > 1.0) {
      e -= 1.0;
    }
    while(e < 0.0) {
      e += 1.0;
    }

    double ew = wristEncoder.get();

    while(ew > 1.0) {
      ew -= 1.0;
    }
    while(ew < 0.0) {
      ew += 1.0;
    }

    if(b > 0.1) {
      armTarget = 0.8;
      wristTarget = 0.8;
    } else if(c > 0.1) {
      armTarget = 0.5;
      wristTarget = 0.5;
    }

    if(armPosition < armTarget && e < 0.8) {
      armPosition += 0.001;
    }
    if(armPosition > armTarget && e > 0.5) {
      armPosition -= 0.001;
    }
    armPID.setPosition(armPosition);

    if(wristPosition < wristTarget && ew < 0.8) {
      wristPosition += 0.001;
    }
    if(wristPosition > wristTarget && ew > 0.5) {
      wristPosition -= 0.001;
    }
    wristPID.setPosition(wristPosition);

    SmartDashboard.putNumber("arm target", wristTarget);
    SmartDashboard.putNumber("arm enc", wristPID.motor.getEncoder().getPosition());
    SmartDashboard.putNumber("armEncoder", ew);
    SmartDashboard.putNumber("armPosition", wristPosition);


    // if(limitSwitch.get() && homeGrabber) {
    //   homeGrabber = false;
    //   grabberPosition = 0;
    //   grabberPositionRamp = 400;
    //   grabberRampFactor = 100;
    // }

    // if(!homeGrabber) {
    //   if(controller.getXButton()) {
    //     grabberPosition = -4500;
    //   } else {
    //     grabberPosition = 0;
    //   }
    // }

    // if(grabberPositionRamp + grabberRampFactor * 5 < grabberPosition) {
    //   grabberPositionRamp += grabberRampFactor;
    // } else if(grabberPositionRamp - grabberRampFactor * 5 > grabberPosition) {
    //   grabberPositionRamp -= grabberRampFactor;
    // }

    
    // if (controller.getStartButton()) {
      swerveDriveTrain.update(controller);
    // }
    
    // wristTarget += controller.getRightX() / 10;

    if (Math.abs(controller.getRightX()) >= .05) {
      grabberPosition += controller.getRightX() * 200;
    }

    
    SmartDashboard.putNumber("DIrection", grabberDirection);
    if (grabberPosition == previousGrabberPosition && limitSwitch.get()) {
      if (grabberDirection == -1 && grabberPosition + 500 < 0) {
        grabberPosition += 250;

      }
    }

    if(grabberPosition > 0 && opened) {
      grabberPosition = 0;
    } else if (grabberPosition < -6000 && opened) {
      grabberPosition = -6000;
    }

    

    //42
    // SmartDashboard.putNumber("Wrist position", wristMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("Wrist target", wristTarget);

    // SmartDashboard.putNumber("grab pos", grabberMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("grab target", grabberPosition);

    SmartDashboard.putBoolean("limit", limitSwitch.get());
    // SmartDashboard.putNumber("Wrist", wristEncoder.get());
    // SmartDashboard.putNumber("WristE", wristMotor.getEncoder().getPosition());

    // if (controller.getAButton()) {
    //   wristTarget = 105;
    // } else if (controller.getBButton()) {
    //   wristTarget = 56;
    // } else {
    //   wristTarget = 85; // rest
    // }
    
    // wristPIDController.setPosition(wristTarget);
    if(!limitSwitch.get() && jLimit) {
      // grabberMotor.setSelectedSensorPosition(125);
      grabberPosition = 0;
      // grabberMotor.set(TalonFXControlMode.Position, 0);
      opened = true;
    }

    if (controller.getBackButton()) {
      // grabberMotor.set(TalonFXControlMode.Position, grabberPosition);
    }

    jLimit = limitSwitch.get();
    grabberDirection = (int) Math.signum(grabberPosition - previousGrabberPosition);
    previousGrabberPosition = grabberPosition;
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
