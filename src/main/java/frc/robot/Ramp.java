package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ramp {
    public final static double MIN_RAMP_LIMIT = -0.23;
    public final static double MID_RAMP_LIMIT = -0.0915;
    public final static double MAX_RAMP_LIMIT = 0;

    private DigitalInput limitSwitch = new DigitalInput(0);
    private boolean jLimit = true;
    private CANSparkMax rampMotor = new CANSparkMax(13, MotorType.kBrushless);
    private SparkMaxPIDController_ rampMotorController = new SparkMaxPIDController_(rampMotor, 255.2, 1.0);
    private double rampPosition = 0;
    private double rampTarget = 0;
    private double rampSpeed = 0.0025;
    
    public Ramp() {

    }

    public boolean isDown() {
        return rampMotorController.getPosition() > -0.005;
    }

    public void setSpeed(double speed) {
        rampSpeed = speed;
    }

    public void setDown() {
        rampTarget = MAX_RAMP_LIMIT;
    }

    public void setDefault() {
        rampTarget = MID_RAMP_LIMIT;
    }

    public void setUp() {
        rampTarget = MIN_RAMP_LIMIT;
    }

    public void oneTimeInit() {
        rampPosition = MIN_RAMP_LIMIT;
        rampTarget = MIN_RAMP_LIMIT;
        rampMotorController.setReferencePosition(rampPosition);
        rampMotorController.setPosition(rampPosition);
        jLimit = true;
    }

    public void sharedInit() {
        oneTimeInit();
    }

    public void autonomousInit() {
        // oneTimeInit();
    }

    public void smartDashboard() {
        SmartDashboard.putBoolean("Ramp Limit", limitSwitch.get());
        SmartDashboard.putNumber("Ramp Encoder Position", rampMotorController.getPosition());
        SmartDashboard.putNumber("Ramp Target", rampTarget);
        SmartDashboard.putNumber("Ramp Position", rampPosition);
    }

    public void update(XboxController controller, XboxController controller2) {
        if (controller2.getPOV() == 180) {
            rampTarget = Math.min(MAX_RAMP_LIMIT, rampTarget + rampSpeed);
        } else if (controller2.getPOV() == 0){
            rampTarget = Math.max(MIN_RAMP_LIMIT, rampTarget - rampSpeed);
        }

        if (rampPosition < rampTarget) {
            rampPosition = Math.min(MAX_RAMP_LIMIT, rampPosition + rampSpeed);
        } else if (rampPosition > rampTarget) {
            rampPosition = Math.max(MIN_RAMP_LIMIT, rampPosition - rampSpeed);
        }
        
        if (Math.abs(rampPosition - rampTarget) < rampSpeed + .001) {
            rampPosition = rampTarget;
        }

        boolean limit = limitSwitch.get();

        if (!limit && jLimit) {
            rampPosition = 0;
            rampTarget = 0;
            rampMotorController.setReferencePosition(rampPosition);
            rampMotorController.setPosition(rampPosition);
            rampSpeed = .003;
        }

        jLimit = limit;

        rampMotorController.setPosition(rampPosition);
    }
}
