package frc.robot.sequence;

import frc.robot.Arm;
import frc.robot.Ramp;

public class ArmHomeSequence extends Sequence {

    public ArmHomeSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.007); arm.setArmSpeed(0.005);}),
            Timer.create(0, () -> arm.setWristPosition(0.07)),
            Timer.create(0, () -> arm.setArmPosition(0.85)),
            Timer.create(500, () -> {}),
            Timer.create(0, () -> arm.setWristPosition(0.026), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .005),
            Timer.create(0, () -> ramp.setDefault(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
            );
    }
    
}
