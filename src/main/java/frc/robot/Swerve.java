package frc.robot;

import java.util.Set;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final SparkMaxPIDController_ motorPIDController2 = new SparkMaxPIDController_(motor2, NEO_CONVERSION_FACTOR, 1, .1413 + .5);
    private final SparkMaxPIDController_ motorPIDController3 = new SparkMaxPIDController_(motor3, NEO_CONVERSION_FACTOR, 1, .0231);
    private final SparkMaxPIDController_ motorPIDController4 = new SparkMaxPIDController_(motor4, NEO_CONVERSION_FACTOR, 1, .9772 - .5);
    
    private Vec2 targetVector = new Vec2();

    private Vec2 motorVector1 = new Vec2();
    private Vec2 motorVector2 = new Vec2();
    private Vec2 motorVector3 = new Vec2();
    private Vec2 motorVector4 = new Vec2();

    private boolean gyroMode = true;
    private boolean jBack = false;
    private double startPigeon = 0;
    private double pigeonAccelAngleOffset = 0;

    private boolean homing;
    private double rotationSpeed = 0.05;
    private double rotation;
    private double rotationTarget;
    private double rotationOutput;

    private double aprilAngleOffset;

    private double kP = 1.0;
    private double kI = 0.0001;
    private double kD = 0;
    
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private double lastP = 0;

    private long lastTime = 0;
    private long currentTime = 0;

    private double botX = 0;
    private double botY = 0;
    private double targetBotX = 0;
    private double targetBotY = 0;
    private boolean homingBot = false;
    public int aprilID = -1;
    private double botSpeedRamp = 0;

    private double botDriveAngle = 0;

    private double encoderDist = 0;
    private double drive1Last = 0;
    private double drive2Last = 0;
    private double drive3Last = 0;
    private double drive4Last = 0;

    //variables for dead reckoning
    private double distToReckon = 0;
    private double angleToReckon = 0;
    private boolean reckoning = false;

    private int[] lastIDs;

    private short[] botAccel = new short[3];

    public Swerve() {
        lastTime = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
    }

    public void robotInit() {
        driveMotor3.setInverted(false);
        driveMotor4.setInverted(true);
    }

    public void sharedInit() {
        this.indWheelController1 = new IndWheelController();
        this.indWheelController2 = new IndWheelController();
        this.indWheelController3 = new IndWheelController();
        this.indWheelController4 = new IndWheelController();

        startPigeon = pigeon.getYaw() * Math.PI / 180;
        pigeonAccelAngleOffset = pigeon.getYaw();

        this.rotation = startPigeon;
        this.rotationTarget = startPigeon;
        this.i = 0;
        this.p = 0;
        this.d = 0;

        this.botSpeedRamp = 0;

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
        Set<String> strings = limelight.getKeys();
        
        for(String s : strings) {
            SmartDashboard.putString(s, "yes");
        }

        double[] pose = limelight.getEntry("botpose").getDoubleArray(defaults);
        double tx = pose[0];
        double ty = pose[1];
        this.botX = tx;
        this.botY = ty;
        this.setBotTarget(6.27, -3.60);
        double drive1 = driveMotor1.getEncoder().getPosition();
        double drive2 = driveMotor1.getEncoder().getPosition();
        double drive3 = driveMotor1.getEncoder().getPosition();
        double drive4 = driveMotor1.getEncoder().getPosition();
        drive1Last = drive1;
        drive2Last = drive2;
        drive3Last = drive3;
        drive4Last = drive4;
        encoderDist = 0;
    }

    public void setHomingBotEnabled(boolean enabled) {
        this.homingBot = enabled;
    }

    public void calibrateBotPosition() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
        Set<String> strings = limelight.getKeys();
        
        for(String s : strings) {
          SmartDashboard.putString(s, "yes");
        }
    
        double[] pose = limelight.getEntry("botpose").getDoubleArray(defaults);
        pigeon.getBiasedAccelerometer(botAccel);
        // pigeon.getAccel
        double tx = pose[0];
        double ty = pose[1];
        double rz = pose[5];

        SmartDashboard.putNumber("tx2", tx);
        SmartDashboard.putNumber("ty2", ty);
        SmartDashboard.putNumber("rz", rz);
        
        // double nz = ((rz > 0 ? rz : (360 + rz)) - 180) * Math.PI / 180;
        // double diff = (pigeon.getYaw() * Math.PI / 180) - startPigeon - nz;
        // startPigeon += diff;
        if(tx != 0 && ty != 0) {
          this.botX = tx;
          this.botY = ty;
          aprilAngleOffset = (pigeon.getYaw() * Math.PI / 180) - startPigeon - (rz * Math.PI / 180);
        }
        // SmartDashboard.putNumber("nz", startPigeon);
        // SmartDashboard.putNumber("diff", diff);
    }

    public void setReckoningTarget(double dist, double angleFromHome) {
        this.distToReckon = dist;
        this.angleToReckon = angleFromHome;
        this.encoderDist = 0;
    }

    public void setReckoningEnabled(boolean enabled) {
        if(enabled) {
            this.encoderDist = 0;
        }
        this.reckoning = enabled;
    }

    public void setBotTarget(double x, double y) {
        this.targetBotX = x;
        this.targetBotY = y;
    }

    //in radians from startPigeon
    public void setRotationTarget(double angle) {
        this.rotationTarget = angle + startPigeon;
    }

    public double distanceSquaredFromBotTarget() {
        return (botY - targetBotY) * (botY - targetBotY) + (botX - targetBotX) * (botX - targetBotX);
    }

    public double distanceFromRotationTarget() {
        return rotationTarget - pigeon.getYaw() * Math.PI / 180;
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
        SmartDashboard.putNumber("botX", botX);
        SmartDashboard.putNumber("botY", botY);
        SmartDashboard.putNumber("botAccelX", botAccel[0]);
        SmartDashboard.putNumber("botAccelY", botAccel[1]);
        SmartDashboard.putNumber("botAccelZ", botAccel[2]);
        SmartDashboard.putNumber("gyroTarget", rotationTarget);
        SmartDashboard.putNumber("rotationOutput", rotationOutput);
    }

    public void calibrateAccelerometer() {
        
    }

    private double wheelRot = 0;

    //helpful for when we need to lube gears in the drivetrain
    public void updateLubeGears(XboxController controller) {
        if(controller.getAButton()) {
            driveMotor1.set(0.03);
            driveMotor2.set(0.03);
            driveMotor3.set(0.03);
            driveMotor4.set(0.03);
        } else {
            driveMotor1.set(0.0);
            driveMotor2.set(0.0);
            driveMotor3.set(0.0);
            driveMotor4.set(0.0);
        }
        if(controller.getBButton()) {
            wheelRot += 0.001;
        }
        motorPIDController1.setPosition(wheelRot);
        motorPIDController2.setPosition(wheelRot);
        motorPIDController3.setPosition(wheelRot);
        motorPIDController4.setPosition(wheelRot);
    }

    private double[] defaults = {0, 0, 0, 0, 0, 0};

    public void update(XboxController controller) {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
        Set<String> strings = limelight.getKeys();
        
        for(String s : strings) {
          SmartDashboard.putString(s, "yes");
        }
    
        double[] pose = limelight.getEntry("botpose").getDoubleArray(defaults);
        aprilID = (int) limelight.getEntry("tid").getDouble(0);
        SmartDashboard.putNumber("Apriltag ID", aprilID);
        pigeon.getBiasedAccelerometer(botAccel);
        // pigeon.getAccel
        double tx = pose[0];
        double ty = pose[1];
        double tz = pose[2];
        double rx = pose[3];
        double ry = pose[4];
        double rz = pose[5];

        SmartDashboard.putNumber("tx2", tx);
        SmartDashboard.putNumber("ty2", ty);
        SmartDashboard.putNumber("rz", rz);
        
        if(tx != 0 && ty != 0 && Math.abs(botX - tx) < 1 && Math.abs(botY - ty) < 1) {
          this.botX = this.botX * 0.2 + tx * 0.8;
          this.botY = this.botY * 0.2 + ty * 0.8;
          aprilAngleOffset = (pigeon.getYaw() * Math.PI / 180) - startPigeon - (rz * Math.PI / 180);
        }

        if(controller.getYButtonPressed()) {
            this.calibrateBotPosition();
        }

        SmartDashboard.putNumber("aprilangleoffset", aprilAngleOffset);

        double drive1 = driveMotor1.getEncoder().getPosition();
        double drive2 = driveMotor1.getEncoder().getPosition();
        double drive3 = driveMotor1.getEncoder().getPosition();
        double drive4 = driveMotor1.getEncoder().getPosition();

        this.encoderDist += (Math.abs(drive1Last - drive1) + Math.abs(drive2Last - drive2) + 
                Math.abs(drive3Last - drive3) + Math.abs(drive4Last - drive4)) / 4;
        
        SmartDashboard.putNumber("drive1Encoder", drive1);
        SmartDashboard.putNumber("drive2Encoder", drive2);
        SmartDashboard.putNumber("drive3Encoder", drive3);
        SmartDashboard.putNumber("drive4Encoder", drive4);
        SmartDashboard.putNumber("encoderDist", encoderDist);

        drive1Last = drive1;
        drive2Last = drive2;
        drive3Last = drive3;
        drive4Last = drive4;

        double delta = (currentTime - lastTime) * 0.001;
        double x = controller.getRightX();
        double y = controller.getRightY();
        double len = Math.sqrt(x * x + y * y);

        if(len < 0.1) {
            x = 0;
            y = 0;
            len = 0;
        }

        if(controller.getBackButton() && homingBot && tx != 0 && ty != 0) {
            double angle = Math.atan2(targetBotX - botX, targetBotY - botY) - aprilAngleOffset;
            double lenSquared = (botY - targetBotY) * (botY - targetBotY) + (botX - targetBotX) * (botX - targetBotX);
            if(lenSquared > 0.04) {
                y = -Math.sin(angle) * 0.6;
                x = -Math.cos(angle) * 0.6;
            } else if(lenSquared > 0.0225) {
                y = -Math.sin(angle) * 0.3;
                x = -Math.cos(angle) * 0.3;
            } else if(lenSquared > 0.0025) {
                y = -Math.sin(angle) * 0.3;
                x = -Math.cos(angle) * 0.3;
            } else {
                y = 0;
                x = 0;
            }
        } else if(controller.getBackButton() && reckoning) {
            if(encoderDist >= distToReckon) {
                reckoning = false;
            } else {
                double angle = angleToReckon - aprilAngleOffset;
                y = -Math.sin(angle) * 0.5;
                x = -Math.cos(angle) * 0.5;
            }
        }

        len = Math.sqrt(x * x + y * y);

        homing = controller.getStartButton();

        double leftX = -controller.getLeftX();

        if(!homing) {
            if(Math.abs(leftX) > 0.1) {
                rotationTarget += rotationSpeed * leftX;
            }
        } else {
            rotationTarget = startPigeon;
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
        SmartDashboard.putNumber("startPigeon", startPigeon);

        double controllerPower = 0.75 * (1.2 * Math.pow(1.043, 100 * len) - 1.2 + .2 * (100 * len)) / 100;//controller.getRightTriggerAxis();

        if(controller.getRightStickButton()) {
            controllerPower *= 0.3;
        }

        if(controllerPower < botSpeedRamp && !homingBot && !reckoning) {
            botSpeedRamp = botSpeedRamp * 0.91 + controllerPower * 0.09;
        } else {
            botSpeedRamp = controllerPower;
        }

        double rotationPower = Math.abs(rotationOutput) * 0.5;

        Vec2 motor1Vec = new Vec2(0, 0);
        Vec2 motor2Vec = new Vec2(0, 0);
        Vec2 motor3Vec = new Vec2(0, 0);
        Vec2 motor4Vec = new Vec2(0, 0);

        if(controllerPower > 0.02) {
            botDriveAngle = Math.atan2(y, x) + (gyroMode ? (pigeon.getYaw() * Math.PI / 180 - startPigeon) : 0);
        }

        if(botSpeedRamp >= 0.02 || Math.abs(rotationOutput) >= 0.1) {
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

            targetVector = new Vec2(botSpeedRamp * Math.cos(botDriveAngle), botSpeedRamp * Math.sin(botDriveAngle));

            if(botSpeedRamp > 0.02) {
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

        driveMotor1.set(-motor1Vec.getLength() * indWheelController1.wheelSign);
        driveMotor2.set(-motor2Vec.getLength() * indWheelController2.wheelSign);
        driveMotor3.set(-motor3Vec.getLength() * indWheelController3.wheelSign);
        driveMotor4.set(-motor4Vec.getLength() * indWheelController4.wheelSign);

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
