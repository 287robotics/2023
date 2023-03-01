package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    // private final TalonFX grabberMotor = new TalonFX(9);
    private CANSparkMax wristMotor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkMaxPIDController_ wristPID = new SparkMaxPIDController_(wristMotor, 100, 0.2); //needs counts
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(7);

    private CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
    private SparkMaxPIDController_ armPID = new SparkMaxPIDController_(armMotor, 125.709, 0.5);
    private CANSparkMax armMotor2 = new CANSparkMax(12, MotorType.kBrushless);
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(9);

    private double armPosition = 0;
    private double armTarget = 0;

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

        lastE = wristPID.getPosition();
        SmartDashboard.putNumber("lastE", lastE);

        // armPID.enableMotor(); //this clearly doesnt work
        // wristPID.enableMotor();

        opened = false;

        homeGrabber = false;
    }

    private double lastE = 0;

    public void update(XboxController controller) {
        double b = controller.getLeftTriggerAxis();
        double c = controller.getRightTriggerAxis();
        boolean gamer = controller.getXButton();
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

        accum = wristPID.getPosition();

        if (gamer) {
            wristTarget = .29;
        } else if (controller.getAButton()) {
            wristTarget = -.2;
        }

        if(b > 0.1) {
            armTarget = 0.7;
        } else if(c > 0.1) {
            armTarget = 0.6;
        }
        
        if(armPosition < armTarget && e < 0.7) {
            armPosition = Math.min(0.7, armPosition + 0.001);
        }
        if(armPosition > armTarget && e > 0.6) {
            armPosition = Math.max(0.6, armPosition - 0.001);
        }
        armPID.setPosition(armPosition);

        if(wristPosition < wristTarget && ew < .3) {
            wristPosition = Math.min(0.3, wristPosition + 0.001);
        }
        if(wristPosition > wristTarget && ew > -.28) {
            wristPosition = Math.max(-0.28, wristPosition - 0.001);
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

        // if(!enabled && enableTime < System.currentTimeMillis()) {
        //     wristPID.enableMotor();
        // }


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
    
}
