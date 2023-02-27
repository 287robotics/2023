package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {
    DutyCycleEncoder absoluteMotorEncoder1 = new DutyCycleEncoder(new DigitalInput(0)); // Strait position: .179
    DutyCycleEncoder absoluteMotorEncoder2 = new DutyCycleEncoder(new DigitalInput(1)); // Strait position: .240
    DutyCycleEncoder absoluteMotorEncoder3 = new DutyCycleEncoder(new DigitalInput(2)); // Strait position: .897
    DutyCycleEncoder absoluteMotorEncoder4 = new DutyCycleEncoder(new DigitalInput(3)); // Strait position: .240

    private final CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax motor3 = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax motor4 = new CANSparkMax(7, MotorType.kBrushless);

    private final CANSparkMax driveMotor1 = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax driveMotor2 = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax driveMotor3 = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax driveMotor4 = new CANSparkMax(8, MotorType.kBrushless);
    private final Pigeon2 pigeon = new Pigeon2(10);
    
    private IndWheelController indWheelController1 = new IndWheelController();
    private IndWheelController indWheelController2 = new IndWheelController();
    private IndWheelController indWheelController3 = new IndWheelController();
    private IndWheelController indWheelController4 = new IndWheelController();

    private final double NEO_CONVERSION_FACTOR = -13.71;
    private final double NEO_DRIVE_CONVERSION_FACTOR = 7.36;

    private final SparkMaxPIDController_ motorPIDController1 = new SparkMaxPIDController_(motor1, NEO_CONVERSION_FACTOR, 1, 0.179 + 0.5 + 0.25);
    private final SparkMaxPIDController_ motorPIDController2 = new SparkMaxPIDController_(motor2, NEO_CONVERSION_FACTOR, 1, 0.240 + 0.5 + 0.25);
    private final SparkMaxPIDController_ motorPIDController3 = new SparkMaxPIDController_(motor3, NEO_CONVERSION_FACTOR, 1, 0.898954 - 0.5 + 0.25);
    private final SparkMaxPIDController_ motorPIDController4 = new SparkMaxPIDController_(motor4, NEO_CONVERSION_FACTOR, 1, 0.963311 - 0.5 + 0.25);

    private Vec2 targetVector = new Vec2();
    private Vec2 motorVector1 = new Vec2();
    private Vec2 motorVector2 = new Vec2();
    private Vec2 motorVector3 = new Vec2();
    private Vec2 motorVector4 = new Vec2();

    private boolean gyroMode = true;
    private boolean jBack = false;
    private double startPigeon = 0;

    private boolean homing;
    private double rotationX;
    private double rotationXTarget;
    
    private double kP = 0.7;
    private double kI = 0.0001;
    private double kD = 0;
    
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private double lastP = 0;

    private long lastTime = 0;
    private long currentTime = 0;

    public Swerve() {
        lastTime = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
    }

    public void robotInit() {
        driveMotor3.setInverted(true);
        driveMotor4.setInverted(true);
    }

    public void sharedInit() {
        startPigeon = pigeon.getYaw() * Math.PI / 180;
    }

    public void calibrateEncoders() {
        motorPIDController1.setReferencePositionNoOffset(absoluteMotorEncoder1.getAbsolutePosition());
        motorPIDController2.setReferencePositionNoOffset(absoluteMotorEncoder2.getAbsolutePosition());
        motorPIDController3.setReferencePositionNoOffset(absoluteMotorEncoder3.getAbsolutePosition());
        motorPIDController4.setReferencePositionNoOffset(absoluteMotorEncoder4.getAbsolutePosition());
    }

    public void update(XboxController controller) {
        double delta = (currentTime - lastTime) * 0.001;
        double x = controller.getRightX();
        double y = controller.getRightY();

        homing = controller.getStartButton();
        
        if(!homing) {
            rotationXTarget = controller.getLeftX();
            p = 0;
            i = 0;
            d = 0;
        } else {
            p = startPigeon - pigeon.getYaw() * Math.PI / 180;
            i += p * delta;
            d = (p - lastP) / delta;
            lastP = p;
            rotationXTarget = -Math.min(1, Math.max(p * kP + i * kI + d * kD, -1));
        }

        rotationX += (rotationXTarget - rotationX) * 0.1;

        SmartDashboard.putNumber("rotationX", rotationX);
        SmartDashboard.putNumber("gyroRot", pigeon.getYaw());

        double angle = Math.atan2(y, x) + (gyroMode ? (pigeon.getYaw() * Math.PI / 180 - startPigeon) : 0);

        double len = Math.sqrt(x * x + y * y);
        double controllerPower = len * len;//controller.getRightTriggerAxis();
        double rotationPower = rotationX * rotationX;

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

            indWheelController1.updateInput(motor1Vec);
            indWheelController2.updateInput(motor2Vec);
            indWheelController3.updateInput(motor3Vec);
            indWheelController4.updateInput(motor4Vec);
        }

        // driveMotor1.set(-motor1Vec.getLength() * indWheelController1.wheelSign);
        // driveMotor2.set(-motor2Vec.getLength() * indWheelController2.wheelSign);
        // driveMotor3.set(-motor3Vec.getLength() * indWheelController3.wheelSign);
        // driveMotor4.set(-motor4Vec.getLength() * indWheelController4.wheelSign);

        indWheelController1.update();
        indWheelController2.update();
        indWheelController3.update();
        indWheelController4.update();

        // motorPIDController1.setPosition(-indWheelController1.wheelPosition / Math.PI / 2 + 0.25);
        // motorPIDController2.setPosition(-indWheelController2.wheelPosition / Math.PI / 2 + 0.25);
        // motorPIDController3.setPosition(-indWheelController3.wheelPosition / Math.PI / 2 + 0.25);
        // motorPIDController4.setPosition(-indWheelController4.wheelPosition / Math.PI / 2 + 0.25);
        
        lastTime = currentTime;
        currentTime = System.currentTimeMillis();
    }
    
}
