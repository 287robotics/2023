package frc.robot.sequence;

import frc.robot.Arm;
import frc.robot.Ramp;

public class ArmHomeSequence extends Sequence {

    public ArmHomeSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.0125); arm.setArmSpeed(0.0075);}),
            Timer.create(0, () -> arm.setGrabberTarget(arm.grabberLimit)),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> {arm.setArmPosition(0.850); arm.setWristSpeed(0.005);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> {ramp.setDefault(); arm.setArmSpeed(0.003);}, () -> arm.getArmPosition() > .73),
            // Timer.create(500, () -> {}),
            Timer.create(0, () -> arm.setWristPosition(0.107), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(0, () -> arm.setGrabberTarget(0)),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
            );
    }
    
}
