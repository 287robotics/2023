package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class Arm {
    private static final Sequence homeSequence = new Sequence(Timer.create(0, () -> {}));
    private static final double ARM_MIN = 0.32; // absolutely farthest should be 0.33
    private static final double ARM_MAX = 0.86; // absolute farthest should be 0.82

    private static final double WRIST_MIN = -0.5;
    private static final double WRIST_MAX = 0.17;

    private final TalonFX grabberMotor = new TalonFX(9);
    private CANSparkMax wristMotor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkMaxPIDController_ wristPID = new SparkMaxPIDController_(wristMotor, 100, 0.5); //needs counts
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(7);

    private CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
    private SparkMaxPIDController_ armPID = new SparkMaxPIDController_(armMotor, 125.709, .7);
    private CANSparkMax armMotor2 = new CANSparkMax(12, MotorType.kBrushless);
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(9);

    private double armPosition = 0;
    private double armTarget = 0;
    private double armSpeed = .004;

    private double wristPosition = 0;
    private double wristTarget = 0;

    private double grabberPosition = 0;
    private double previousGrabberPosition = 0;
    private double grabberPositionRamp = 0;
    private double grabberRampFactor = 100;
    private boolean homeGrabber = false;
    private boolean opened = false;
    private int grabberDirection = 1;

    private long enableTime = 0;
    private boolean enabled = false;

    private double accum = 0;
    
    private DigitalInput limitSwitch = new DigitalInput(6); //pressed = false
    private boolean jLimit = true;

    public Arm() {

    }

    public void sharedInit() {
        enableTime = System.currentTimeMillis() + 2000;
        armMotor2.follow(armMotor, true);
        armPID.disableMotor();
        wristPID.disableMotor();
        grabberPosition = -10000;
        double e = armEncoder.get();

        while (e > 1.0) {
            e -= 1.0;
        }
        
        while (e < 0.0) {
            e += 1.0;
        }

        double ew = wristEncoder.get();

        while (ew > 0.5) {
            ew -= 1.0;
        }
        
        while (ew < -0.5) {
            ew += 1.0;
        }

        armPosition = e;
        armTarget = e;
        armPosition = e;
        armTarget = e;
        armPID.setReferencePosition(e);
        armPID.setPosition(e);

        wristPosition = ew;
        wristTarget = ew;
        wristPID.setReferencePosition(ew);
        wristPID.setPosition(ew);

        lastE = armPID.getPosition();
        SmartDashboard.putNumber("lastE", lastE);

        armPID.enableMotor(); //this clearly doesnt work
        wristPID.enableMotor();

        opened = false;

        homeGrabber = false;
    }

    private double lastE = 0;

    public void update(XboxController controller, XboxController controller2) {
        double ry = controller2.getRightY();
        double ly = controller2.getLeftY();
        double e = armEncoder.get();
        double ew = wristEncoder.get();
        
        while(ew > 0.5) {
            ew -= 1.0;
        }
        while(ew < -0.5) {
            ew += 1.0;
        }
        
        while(e > 1.0) {
            e -= 1.0;
        }
        while(e < 0.0) {
            e += 1.0;
        }

        accum = armPID.getPosition();

        if (controller.getXButton()) {

        } else if (controller.getAButton()) {

        }

        if(ry > 0.1) {
            armTarget = Math.min(ARM_MAX, armTarget + ry * .004);
        } else if (ry < -0.1) {
            armTarget = Math.max(ARM_MIN, armTarget + ry * .004);
        }

        if (ly < -.1) {
            wristTarget = Math.min(WRIST_MAX, wristTarget - ly * .004);
        } else if (ly > .1) {
            wristTarget = Math.max(WRIST_MIN, wristTarget - ly * .004);

            
        }
        
        if(armPosition < armTarget && e < ARM_MAX) {
            armPosition = Math.min(ARM_MAX, armPosition + armSpeed);
        }
        if(armPosition > armTarget && e > ARM_MIN) {
            armPosition = Math.max(ARM_MIN, armPosition - armSpeed);
        }
        armPID.setPosition(armPosition);

        if(wristPosition < wristTarget && ew < WRIST_MAX) {
            wristPosition = Math.min(WRIST_MAX, wristPosition + 0.004);
        }
        if(wristPosition > wristTarget && ew > WRIST_MIN) {
            wristPosition = Math.max(WRIST_MIN, wristPosition - 0.004);
        }

        wristPID.setPosition(wristPosition);

        SmartDashboard.putNumber("wristEncoder", wristPID.getPosition());
        SmartDashboard.putNumber("wristPosition", wristPosition);
        SmartDashboard.putNumber("wristTarget", wristTarget);
        SmartDashboard.putNumber("wristEncoderNormalized", ew);

        SmartDashboard.putNumber("armEncoder", armEncoder.get());
        SmartDashboard.putNumber("armEncoderNormalized", e);
        SmartDashboard.putNumber("armPosition", armPosition);
        SmartDashboard.putNumber("armTarget", armTarget);

        SmartDashboard.putNumber("Accumulator", accum);

        if(!enabled && enableTime < System.currentTimeMillis()) {
            wristPID.enableMotor();
        }

        if (Math.abs(controller2.getRightTriggerAxis()) >= .05) {
            grabberPosition += controller2.getRightTriggerAxis() * 200;
        }

        if (Math.abs(controller2.getLeftTriggerAxis()) >= .05) {
            grabberPosition -= controller2.getLeftTriggerAxis() * 200;
        }

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

        if(!limitSwitch.get() && jLimit) {
            grabberMotor.setSelectedSensorPosition(125);
            grabberPosition = 0;
            grabberMotor.set(TalonFXControlMode.Position, 0);
            opened = true;
          }
      
          if (controller.getBackButton()) {
            grabberMotor.set(TalonFXControlMode.Position, grabberPosition);
          }
      
          jLimit = limitSwitch.get();
          grabberDirection = (int) Math.signum(grabberPosition - previousGrabberPosition);
          previousGrabberPosition = grabberPosition;
    }
    
}
