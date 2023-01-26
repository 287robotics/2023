// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  DutyCycleEncoder absoluteMotorEncoder1 = new DutyCycleEncoder(new DigitalInput(0)); // Strait position: .179
  DutyCycleEncoder absoluteMotorEncoder2 = new DutyCycleEncoder(new DigitalInput(1)); // Strait position: .240
  DutyCycleEncoder absoluteMotorEncoder3 = new DutyCycleEncoder(new DigitalInput(2)); // Strait position: .897
  DutyCycleEncoder absoluteMotorEncoder4 = new DutyCycleEncoder(new DigitalInput(3)); // Strait position: .240
  
  private DigitalInput limitSwitch = new DigitalInput(4);

  // private final TalonFX grabberMotor = new TalonFX(9);
  private final Pigeon2 pigeon = new Pigeon2(10);

  private double grabberPosition = 0;
  private double grabberPositionRamp = 0;
  private double grabberRampFactor = 100;
  private boolean homeGrabber = false;

  private final CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax motor2 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax motor3 = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax motor4 = new CANSparkMax(7, MotorType.kBrushless);

  private final CANSparkMax driveMotor1 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax driveMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax driveMotor3 = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax driveMotor4 = new CANSparkMax(8, MotorType.kBrushless);

  private final double NEO_CONVERSION_FACTOR = -13.71;
  private final double NEO_DRIVE_CONVERSION_FACTOR = 7.36;

  private double wheel1DirTarget = 0;
  private double wheel2DirTarget = 0;
  private double wheel3DirTarget = 0;
  private double wheel4DirTarget = 0;

  private double wheel1DirActual = 0;
  private double wheel2DirActual = 0;
  private double wheel3DirActual = 0;
  private double wheel4DirActual = 0;

  private double wheel1Position = 0;
  private double wheel2Position = 0;
  private double wheel3Position = 0;
  private double wheel4Position = 0;
  
  private double wheel1Sign = 1;
  private double wheel2Sign = 1;
  private double wheel3Sign = 1;
  private double wheel4Sign = 1;

  private final SparkMaxPIDController_ motorPIDController1 = new SparkMaxPIDController_(motor1, NEO_CONVERSION_FACTOR, 1, 0.179 + 0.5 + 0.25);
  private final SparkMaxPIDController_ motorPIDController2 = new SparkMaxPIDController_(motor2, NEO_CONVERSION_FACTOR, 1, 0.240 + 0.5 + 0.25);
  private final SparkMaxPIDController_ motorPIDController3 = new SparkMaxPIDController_(motor3, NEO_CONVERSION_FACTOR, 1, 0.898954 - 0.5 + 0.25);
  private final SparkMaxPIDController_ motorPIDController4 = new SparkMaxPIDController_(motor4, NEO_CONVERSION_FACTOR, 1, 0.963311 - 0.5 + 0.25);

  // private final SparkMaxPIDController_ driveMotorPIDController1 = new SparkMaxPIDController_(driveMotor1, NEO_DRIVE_CONVERSION_FACTOR, .25, 0);
  // private final SparkMaxPIDController_ driveMotorPIDController2 = new SparkMaxPIDController_(driveMotor2, NEO_DRIVE_CONVERSION_FACTOR, .25, 0);
  // private final SparkMaxPIDController_ driveMotorPIDController3 = new SparkMaxPIDController_(driveMotor3, NEO_DRIVE_CONVERSION_FACTOR, .25, 0);
  // private final SparkMaxPIDController_ driveMotorPIDController4 = new SparkMaxPIDController_(driveMotor4, NEO_DRIVE_CONVERSION_FACTOR, .25, 0);

  private Vec2 targetVector = new Vec2();
  private Vec2 motorVector1 = new Vec2();
  private Vec2 motorVector2 = new Vec2();
  private Vec2 motorVector3 = new Vec2();
  private Vec2 motorVector4 = new Vec2();

  private double startPigeon = 0;
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

    driveMotor3.setInverted(true);
    driveMotor4.setInverted(true);

    // driveMotorPIDController1.setControlType(CANSparkMax.ControlType.kVoltage);
    // driveMotorPIDController2.setControlType(CANSparkMax.ControlType.kVoltage); 
    // driveMotorPIDController3.setControlType(CANSparkMax.ControlType.kVoltage);
    // driveMotorPIDController4.setControlType(CANSparkMax.ControlType.kVoltage);

    System.out.println(absoluteMotorEncoder1.isConnected());

  }

  //Calibrates motors so that their relative encoders agree with absolute encoders
  private void calibrateEncoders() {
    motorPIDController1.setReferencePositionNoOffset(absoluteMotorEncoder1.getAbsolutePosition());
    motorPIDController2.setReferencePositionNoOffset(absoluteMotorEncoder2.getAbsolutePosition());
    motorPIDController3.setReferencePositionNoOffset(absoluteMotorEncoder3.getAbsolutePosition());
    motorPIDController4.setReferencePositionNoOffset(absoluteMotorEncoder4.getAbsolutePosition());
  }

  private void sharedInit() {
    calibrateEncoders();
    wheel1Position = 0;
    wheel2Position = 0;
    wheel3Position = 0;
    wheel4Position = 0;
    startPigeon = pigeon.getYaw() / 360;
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

  private double mod(double a, double n) {
    return a - Math.floor(a / n) * n;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //rotations of motor / 13.75 = rotations of swerve
    SmartDashboard.putNumber("Motor 1 Absolute Position", absoluteMotorEncoder1.getAbsolutePosition());
    SmartDashboard.putNumber("Motor 2 Absolute Position", absoluteMotorEncoder2.getAbsolutePosition());
    SmartDashboard.putNumber("Motor 3 Absolute Position", absoluteMotorEncoder3.getAbsolutePosition());
    SmartDashboard.putNumber("Motor 4 Absolute Position", absoluteMotorEncoder4.getAbsolutePosition());

    SmartDashboard.putNumber("Motor 1 Position", motorPIDController1.motor.getEncoder().getPosition());
    SmartDashboard.putNumber("Motor 2 Position", motorPIDController2.motor.getEncoder().getPosition());

    
    double yaw = pigeon.getYaw();
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Mapped",pigeon.getYaw() / 360 - startPigeon);
    
    double x = controller.getRightX();
    double y = controller.getRightY();
    double rotationX = controller.getLeftX();

    double angle = Math.atan2(y, x);
    double len = Math.sqrt(x * x + y * y);

    double controllerPower = len * len;//controller.getRightTriggerAxis();
    double rotationPower = rotationX * rotationX;

    double wheelChange1 = 0;
    double wheelChange2 = 0;
    double wheelChange3 = 0;
    double wheelChange4 = 0;

    Vec2 motor1Vec = new Vec2(0, 0);
    Vec2 motor2Vec = new Vec2(0, 0);
    Vec2 motor3Vec = new Vec2(0, 0);
    Vec2 motor4Vec = new Vec2(0, 0);

    if(len >= 0.1 || Math.abs(rotationX) >= 0.1) {
      if (rotationX <= -.1) {
        motorVector1 = new Vec2(-rotationPower * Math.cos(Math.PI / 4), -rotationPower * Math.sin(Math.PI / 4));
        motorVector2 = new Vec2(rotationPower * Math.cos(3 * Math.PI / 4), rotationPower * Math.sin(3 * Math.PI / 4));
        motorVector3 = new Vec2(-rotationPower * Math.cos(-3 * Math.PI / 4), -rotationPower * Math.sin(-3 * Math.PI / 4));
        motorVector4 = new Vec2(rotationPower * Math.cos(-Math.PI / 4), rotationPower * Math.sin(-Math.PI / 4));
        
      } else if (rotationX >= .1) {
        motorVector1 = new Vec2(-rotationPower * Math.cos(-Math.PI / 4 - Math.PI / 2), -rotationPower * Math.sin(-Math.PI / 4 - Math.PI / 2));
        motorVector2 = new Vec2(rotationPower * Math.cos(Math.PI / 4 - Math.PI / 2), rotationPower * Math.sin(Math.PI / 4 - Math.PI / 2));
        motorVector3 = new Vec2(-rotationPower * Math.cos(-3 * Math.PI / 4 - Math.PI), -rotationPower * Math.sin(-3 * Math.PI / 4 - Math.PI));
        motorVector4 = new Vec2(rotationPower * Math.cos(5 * Math.PI / 4 - Math.PI / 2), rotationPower * Math.sin(5 * Math.PI / 4 - Math.PI / 2));
        
      } else {
        motorVector1 = new Vec2(0, 0);
        motorVector2 = new Vec2(0, 0);
        motorVector3 = new Vec2(0, 0);
        motorVector4 = new Vec2(0, 0);
      }

      targetVector = new Vec2(controllerPower * Math.cos(angle), controllerPower * Math.sin(angle));

      if(len > 0.1) {
        motor1Vec = targetVector.add(motorVector1).limitLength(1);
        motor2Vec = targetVector.add(motorVector2).limitLength(1);
        motor3Vec = targetVector.add(motorVector3).limitLength(1);
        motor4Vec = targetVector.add(motorVector4).limitLength(1);
      } else {
        motor1Vec = motorVector1.limitLength(1);
        motor2Vec = motorVector2.limitLength(1);
        motor3Vec = motorVector3.limitLength(1);
        motor4Vec = motorVector4.limitLength(1);
      }

      wheel1DirTarget = motor1Vec.toRadians();
      wheel2DirTarget = motor2Vec.toRadians();
      wheel3DirTarget = motor3Vec.toRadians();
      wheel4DirTarget = motor4Vec.toRadians();

      double normalizedPosition1 = Math.IEEEremainder(wheel1Position, Math.PI * 2);
      double normalizedPosition2 = Math.IEEEremainder(wheel2Position, Math.PI * 2);
      double normalizedPosition3 = Math.IEEEremainder(wheel3Position, Math.PI * 2);
      double normalizedPosition4 = Math.IEEEremainder(wheel4Position, Math.PI * 2);

      double angleDiff1 = Math.PI - Math.abs(Math.abs(wheel1DirTarget - normalizedPosition1) - Math.PI);
      double angleDiff2 = Math.PI - Math.abs(Math.abs(wheel2DirTarget - normalizedPosition2) - Math.PI);
      double angleDiff3 = Math.PI - Math.abs(Math.abs(wheel3DirTarget - normalizedPosition3) - Math.PI);
      double angleDiff4 = Math.PI - Math.abs(Math.abs(wheel4DirTarget - normalizedPosition4) - Math.PI);

      //needs a whole ton of work because im pretty sure this dont work
      
      if(angleDiff1 > Math.PI / 2) {
        if(wheel1DirTarget > 0) {
          wheel1DirActual = wheel1DirTarget - Math.PI;
        } else {
          wheel1DirActual = wheel1DirTarget + Math.PI;
        }
        wheel1Sign = -1;
      } else {
        wheel1Sign = 1;
        wheel1DirActual = wheel1DirTarget;
      }

      if(angleDiff2 > Math.PI / 2) {
        if(wheel2DirTarget > 0) {
          wheel2DirActual = wheel2DirTarget - Math.PI;
        } else {
          wheel2DirActual = wheel2DirTarget + Math.PI;
        }
        wheel2Sign = -1;
      } else {
        wheel2Sign = 1;
        wheel2DirActual = wheel2DirTarget;
      }

      if(angleDiff3 > Math.PI / 2) {
        if(wheel3DirTarget > 0) {
          wheel3DirActual = wheel3DirTarget - Math.PI;
        } else {
          wheel3DirActual = wheel3DirTarget + Math.PI;
        }
        wheel3Sign = -1;
      } else {
        wheel3Sign = 1;
        wheel3DirActual = wheel3DirTarget;
      }

      if(angleDiff4 > Math.PI / 2) {
        if(wheel4DirTarget > 0) {
          wheel4DirActual = wheel4DirTarget - Math.PI;
        } else {
          wheel4DirActual = wheel4DirTarget + Math.PI;
        }
        wheel4Sign = -1;
      } else {
        wheel4Sign = 1;
        wheel4DirActual = wheel4DirTarget;
      }

      wheelChange1 = mod(((wheel1DirActual - normalizedPosition1) + Math.PI), Math.PI * 2) - Math.PI;
      wheelChange2 = mod(((wheel2DirActual - normalizedPosition2) + Math.PI), Math.PI * 2) - Math.PI;
      wheelChange3 = mod(((wheel3DirActual - normalizedPosition3) + Math.PI), Math.PI * 2) - Math.PI;
      wheelChange4 = mod(((wheel4DirActual - normalizedPosition4) + Math.PI), Math.PI * 2) - Math.PI;
      SmartDashboard.putNumber("wheel change 1", angleDiff1);
    }

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

    // grabberMotor.set(TalonFXControlMode.Position, rotationX * 1000);
        
    driveMotor1.set(-motor1Vec.getLength() * wheel1Sign);
    driveMotor2.set(-motor2Vec.getLength() * wheel2Sign);
    driveMotor3.set(-motor3Vec.getLength() * wheel3Sign);
    driveMotor4.set(-motor4Vec.getLength() * wheel4Sign);

    wheel1Position += wheelChange1;
    wheel2Position += wheelChange2;
    wheel3Position += wheelChange3;
    wheel4Position += wheelChange4;

    // System.out.println(targetVector.add(motorVector1).limit(1, 0).toAngle() / 360);
    motorPIDController1.setPosition(-wheel1Position / Math.PI / 2 + 0.25);
    motorPIDController2.setPosition(-wheel2Position / Math.PI / 2 + 0.25);
    motorPIDController3.setPosition(-wheel3Position / Math.PI / 2 + 0.25);
    motorPIDController4.setPosition(-wheel4Position / Math.PI / 2 + 0.25);

    SmartDashboard.putNumber("Position", wheel1Position);
    // grabberMotor.set(TalonFXControlMode.Position, grabberPositionRamp);
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
