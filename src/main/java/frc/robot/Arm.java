package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sequence.ArmHomeSequence;
import frc.robot.sequence.DunkSequence;

public class Arm {

    private static final double ARM_MIN = 0.32; // absolutely farthest should be 0.33
    private static final double ARM_MAX = 0.87; // absolute farthest should be 0.82

    private static final double WRIST_MIN = -0.5;
    private static final double WRIST_MAX = 0.2;

    private TalonFX grabberMotor = new TalonFX(9);
    private CANSparkMax wristMotor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkMaxPIDController_ wristPID = new SparkMaxPIDController_(wristMotor, 100, 1.0); //needs counts
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(7);

    private CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
    private SparkMaxPIDController_ armPID = new SparkMaxPIDController_(armMotor, 125.709, 1.0);
    private CANSparkMax armMotor2 = new CANSparkMax(12, MotorType.kBrushless);
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(9);

    private double armPosition = 0;
    private double armTarget = 0;
    private double armSpeed = .004;

    private double wristPosition = 0;
    private double wristTarget = 0;
    private double wristSpeed = .004;

    private double grabberPosition = 0;
    private double previousGrabberPosition = 0;
    private boolean opened = false;
    private int grabberDirection = 1;
    private double grabberLimit = -6350;

    private double accum = 0;
    
    private DigitalInput limitSwitch = new DigitalInput(6); //pressed = false
    private boolean jLimit = true;

    public Arm() {
        
    }

    public void sharedInit() {
        armMotor2.follow(armMotor, true);
        armPID.disableMotor();
        wristPID.disableMotor();

        grabberMotor = new TalonFX(9);

        grabberPosition = -10000;
        grabberMotor.setSelectedSensorPosition(-10000);
        grabberMotor.set(ControlMode.Position, -10000);

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
    }

    private double lastE = 0;

    public void setArmSpeed(double armSpeed) {
        this.armSpeed = armSpeed;
    }

    public void setArmPosition(double armPos) {
        this.armTarget = armPos;
    }

    public void setWristSpeed(double armSpeed) {
        this.wristSpeed = armSpeed;
    }

    public void setWristPosition(double armPos) {
        this.wristTarget = armPos;
    }

    public double getWristPosition() {
        return wristPosition;
    }

    public double getWristTarget() {
        return wristTarget;
    }

    public double getArmPosition() {
        return armPosition;
    }

    public double getArmTarget() {
        return armTarget;
    }
    public void update(XboxController controller, XboxController controller2) {
        if(Robot.obj.areSequencesComplete()) {
            double ry = controller2.getRightY();
            double ly = controller2.getLeftY();

            if(ry > 0.1) {
                armTarget = Math.min(ARM_MAX, armTarget + ry * armSpeed);
            } else if (ry < -0.1) {
                armTarget = Math.max(ARM_MIN, armTarget + ry * armSpeed);
            }

            if (ly < -.1) {
                wristTarget = Math.min(WRIST_MAX, wristTarget - ly * wristSpeed);
            } else if (ly > .1) {
                wristTarget = Math.max(WRIST_MIN, wristTarget - ly * wristSpeed);
            }
        
            if (Math.abs(controller2.getRightTriggerAxis()) >= .05) {
                grabberPosition += controller2.getRightTriggerAxis() * 200;
            }

            if (Math.abs(controller2.getLeftTriggerAxis()) >= .05) {
                grabberPosition -= controller2.getLeftTriggerAxis() * 200;
            }

        }
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

        if(armPosition < armTarget && e < ARM_MAX) {
            armPosition = Math.min(ARM_MAX, armPosition + armSpeed);
        }
        if(armPosition > armTarget && e > ARM_MIN) {
            armPosition = Math.max(ARM_MIN, armPosition - armSpeed);
        }
        armPID.setPosition(armPosition);

        if(wristPosition < wristTarget && ew < WRIST_MAX) {
            wristPosition = Math.min(WRIST_MAX, wristPosition + wristSpeed);
        }
        if(wristPosition > wristTarget && ew > WRIST_MIN) {
            wristPosition = Math.max(WRIST_MIN, wristPosition - wristSpeed);
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

        if (grabberPosition == previousGrabberPosition && limitSwitch.get()) {
            if (grabberDirection == -1 && grabberPosition + 750 < 0) {
                grabberPosition = grabberMotor.getSelectedSensorPosition();
            }
        }

        if(grabberPosition > 0 && opened) {
            grabberPosition = 0;
        } else if (grabberPosition < grabberLimit && opened) {
            grabberPosition = grabberLimit;
        }

        if(!limitSwitch.get() && jLimit) {
            grabberMotor.setSelectedSensorPosition(125);
            grabberPosition = 0;
            grabberMotor.set(TalonFXControlMode.Position, 0);
            opened = true;
        }
    
        grabberMotor.set(TalonFXControlMode.Position, grabberPosition);

        SmartDashboard.putNumber("grabberPosition", grabberPosition);
    
        jLimit = limitSwitch.get();
        grabberDirection = (int) Math.signum(grabberPosition - previousGrabberPosition);
        previousGrabberPosition = grabberPosition;
    }
}
