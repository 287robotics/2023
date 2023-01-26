package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;


public class SparkMaxPIDController_ {
    public CANSparkMax motor;
    public double countsPerRotation = 1;
    public double offset = 0;
    public SparkMaxPIDController pidController;

    public double kP = 0.2;
    public double kI = 0.001;
    public double kD = 0;
    public double kIz = 0;
    public double kFF = 0;
    public double kMaxOutput = 1;
    public double kMinOutput = -1;

    public CANSparkMax.ControlType controlType = CANSparkMax.ControlType.kPosition;

    public double clamp(double n, double max, double min) {
        return Math.min(Math.max(n, min), max);
    }

    public SparkMaxPIDController_(CANSparkMax motor) {
        this(motor, 1, 1, 0);
    }

    public SparkMaxPIDController_(CANSparkMax motor, double countsPerRotation) {
        this(motor, countsPerRotation, 1, 0);
    }

    public SparkMaxPIDController_(CANSparkMax motor, double countsPerRotation, double maxSpeed) {
        this(motor, countsPerRotation, maxSpeed, 0);
    }

    public void setControlType(CANSparkMax.ControlType type) {
        controlType = type;
    }

    public SparkMaxPIDController_(CANSparkMax motor, double countsPerRotation, double maxSpeed, double offset) {
        this.motor = motor;
        this.pidController = motor.getPIDController();
        this.countsPerRotation = countsPerRotation;
        this.kMaxOutput = maxSpeed;
        this.kMinOutput = -maxSpeed;
        this.offset = offset;
        setVariables(kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
    }

    public void setVariables(double kP_, double kI_, double kD_, double kIz_, double kFF_, double kMaxOutput_, double kMinOutput_) {
        kP = kP_;
        kI = kI_;
        kD = kD_;
        kIz = kIz_;
        kFF = kFF_;
        kMaxOutput = kMaxOutput_;
        kMinOutput = kMinOutput_;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void disableMotor() {
        pidController.setOutputRange(0, 0);
    }

    public void enableMotor() {
        pidController.setOutputRange(this.kMinOutput, this.kMaxOutput);
    }

    public double getPosition() {
        return this.motor.getEncoder().getPosition() / countsPerRotation - offset;
    }

    public double getNormalizedRadianPosition() {
        double raw = this.motor.getEncoder().getPosition() / countsPerRotation - offset;
        return Math.IEEEremainder(raw, 1) * Math.PI * 2;
    }

    public void setReferencePosition(double positionInRotations) {
        motor.getEncoder().setPosition((positionInRotations + offset) * countsPerRotation);
    }

    public void setReferencePositionNoOffset(double positionInRotations) {
        motor.getEncoder().setPosition(positionInRotations * countsPerRotation);
    }

    public void setReference(double n) {
        pidController.setReference(n * countsPerRotation, controlType);
    }

    public void setPosition(double positionInRotations) {
            
        pidController.setReference((positionInRotations + offset) * countsPerRotation, controlType);
    }

    public void setPositionCounts(double position) {
        pidController.setReference(position + offset * countsPerRotation, controlType);
    }
}

