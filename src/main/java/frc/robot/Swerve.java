package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {
    CANCoder absMotorEncoder1 = new CANCoder(15);
    CANCoder absMotorEncoder2 = new CANCoder(16);
    CANCoder absMotorEncoder3 = new CANCoder(17);
    CANCoder absMotorEncoder4 = new CANCoder(14);

    private final CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax motor3 = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax motor4 = new CANSparkMax(5, MotorType.kBrushless);

    private final CANSparkMax driveMotor1 = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax driveMotor2 = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax driveMotor3 = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax driveMotor4 = new CANSparkMax(6, MotorType.kBrushless);
    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(10);
    
    private IndWheelController indWheelController1 = new IndWheelController();
    private IndWheelController indWheelController2 = new IndWheelController();
    private IndWheelController indWheelController3 = new IndWheelController();
    private IndWheelController indWheelController4 = new IndWheelController();

    private final double NEO_CONVERSION_FACTOR = -13.71;
    private final double NEO_DRIVE_CONVERSION_FACTOR = 7.36;

    private final SparkMaxPIDController_ motorPIDController1 = new SparkMaxPIDController_(motor1, NEO_CONVERSION_FACTOR, 1, .3 - .5); // WHAT THE FUCK
    private final SparkMaxPIDController_ motorPIDController2 = new SparkMaxPIDController_(motor2, NEO_CONVERSION_FACTOR, 1, .652);
    private final SparkMaxPIDController_ motorPIDController3 = new SparkMaxPIDController_(motor3, NEO_CONVERSION_FACTOR, 1, .519 - .5);
    private final SparkMaxPIDController_ motorPIDController4 = new SparkMaxPIDController_(motor4, NEO_CONVERSION_FACTOR, 1, .985 - .5);
    
    private Vec2 targetVector = new Vec2();

    private Vec2 motorVector1 = new Vec2();
    private Vec2 motorVector2 = new Vec2();
    private Vec2 motorVector3 = new Vec2();
    private Vec2 motorVector4 = new Vec2();

    private boolean gyroMode = true;
    private boolean jBack = false;
    private double startPigeon = 0;

    private boolean homing;
    private double rotationSpeed = 0.05;
    private double rotation;
    private double rotationTarget;
    private double rotationOutput;

    private short[] accel = new short[3];

    private short[] offsets = new short[3];

    private double ax = 0;
    private double ay = 0;
    private double az = 0;

    private double vx = 0;
    private double vy = 0;
    private double vz = 0;

    private double px = 0;
    private double py = 0;
    private double pz = 0;

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
        driveMotor3.setInverted(false);
        driveMotor4.setInverted(true);
    }

    public void sharedInit() {
        startPigeon = pigeon.getYaw() * Math.PI / 180;
        this.rotation = startPigeon;
        this.rotationTarget = startPigeon;
        this.ax = 0;
        this.ay = 0;
        this.az = 0;
    }

    public void calibrateEncoders() {
        motorPIDController1.setReferencePositionNoOffset(absMotorEncoder1.getAbsolutePosition() / 360);
        motorPIDController2.setReferencePositionNoOffset(absMotorEncoder2.getAbsolutePosition() / 360);
        motorPIDController3.setReferencePositionNoOffset(absMotorEncoder3.getAbsolutePosition() / 360);
        motorPIDController4.setReferencePositionNoOffset(absMotorEncoder4.getAbsolutePosition() / 360);
    }

    public void smartDashboard() {
        SmartDashboard.putNumber("absMotorEncoder1", absMotorEncoder1.getAbsolutePosition() / 360);
        SmartDashboard.putNumber("absMotorEncoder2", absMotorEncoder2.getAbsolutePosition() / 360);
        SmartDashboard.putNumber("absMotorEncoder3", absMotorEncoder3.getAbsolutePosition() / 360);
        SmartDashboard.putNumber("absMotorEncoder4", absMotorEncoder4.getAbsolutePosition() / 360);
        SmartDashboard.putNumber("relMotorEncoder1", motorPIDController1.motor.getEncoder().getPosition());
        SmartDashboard.putNumber("relMotorEncoder2", motorPIDController2.motor.getEncoder().getPosition());
        SmartDashboard.putNumber("relMotorEncoder3", motorPIDController3.motor.getEncoder().getPosition());
        SmartDashboard.putNumber("relMotorEncoder4", motorPIDController4.motor.getEncoder().getPosition());
    }

    public void calibrateAccelerometer() {
        for(int i = 0; i < 3; i++) {
            this.offsets[i] = (short) -this.accel[i];
        }
    }

    public void update(XboxController controller) {
        double delta = (currentTime - lastTime) * 0.001;
        double x = controller.getRightX();
        double y = controller.getRightY();

        homing = controller.getStartButton();

        double leftX = controller.getLeftX();

        if(!homing) {
            rotationTarget += rotationSpeed * leftX;
        } else {
            rotationTarget = startPigeon;
        }

        if(controller.getBackButtonPressed()) {
            this.calibrateAccelerometer();
        }

        if (rotation < rotationTarget) {
            rotation += rotationSpeed;
        } else if (rotation > rotationTarget) {
            rotation -= rotationSpeed;
        }
        p = rotation - pigeon.getYaw() * Math.PI / 180;
        i += p * delta;
        d = (p - lastP) / delta;
        rotationOutput = -Math.min(1, Math.max(p * kP + i * kI + d * kD, -1));

        lastP = p;

        SmartDashboard.putNumber("rotationX", rotation);
        SmartDashboard.putNumber("gyroRot", pigeon.getYaw());

        pigeon.getBiasedAccelerometer(accel);
        ax = (accel[0] + offsets[0]) * 9.8 / 16384.0;
        ay = (accel[1] + offsets[1]) * 9.8 / 16384.0;
        az = (accel[2] + offsets[2]) * 9.8 / 16384.0;

        SmartDashboard.putNumber("posX", ax);
        SmartDashboard.putNumber("posY", ay);
        SmartDashboard.putNumber("posZ", az);

        double angle = Math.atan2(y, x) + (gyroMode ? (pigeon.getYaw() * Math.PI / 180 - startPigeon) : 0);

        double len = Math.sqrt(x * x + y * y);
        double controllerPower = len * len;//controller.getRightTriggerAxis();
        double rotationPower = Math.abs(rotationOutput); //needs beeg testing

        Vec2 motor1Vec = new Vec2(0, 0);
        Vec2 motor2Vec = new Vec2(0, 0);
        Vec2 motor3Vec = new Vec2(0, 0);
        Vec2 motor4Vec = new Vec2(0, 0);

        if(len >= 0.1 || Math.abs(rotationOutput) >= 0.1) {
            if (rotationOutput <= -.1) {
                motorVector1 = new Vec2(-rotationPower * Math.cos(Math.PI / 4), -rotationPower * Math.sin(Math.PI / 4));
                motorVector2 = new Vec2(rotationPower * Math.cos(3 * Math.PI / 4), rotationPower * Math.sin(3 * Math.PI / 4));
                motorVector3 = new Vec2(-rotationPower * Math.cos(-3 * Math.PI / 4), -rotationPower * Math.sin(-3 * Math.PI / 4));
                motorVector4 = new Vec2(rotationPower * Math.cos(-Math.PI / 4), rotationPower * Math.sin(-Math.PI / 4));
                
            } else if (rotationOutput >= .1) {
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

        driveMotor1.set(-motor1Vec.getLength() * indWheelController1.wheelSign / 2);
        driveMotor2.set(-motor2Vec.getLength() * indWheelController2.wheelSign / 2);
        driveMotor3.set(-motor3Vec.getLength() * indWheelController3.wheelSign / 2);
        driveMotor4.set(-motor4Vec.getLength() * indWheelController4.wheelSign / 2);

        indWheelController1.update();
        indWheelController2.update();
        indWheelController3.update();
        indWheelController4.update();

        motorPIDController1.setPosition(-indWheelController1.wheelPosition / Math.PI / 2 + 0.25);
        motorPIDController2.setPosition(-indWheelController2.wheelPosition / Math.PI / 2 + 0.25);
        motorPIDController3.setPosition(-indWheelController3.wheelPosition / Math.PI / 2 + 0.25);
        motorPIDController4.setPosition(-indWheelController4.wheelPosition / Math.PI / 2 + 0.25);
        
        lastTime = currentTime;
        currentTime = System.currentTimeMillis();
    }
    
}
