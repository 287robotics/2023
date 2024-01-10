package frc.robot.sequence.cone;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class FloorSequence extends Sequence {
    public FloorSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.006); arm.setArmSpeed(0.006);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.7052)),
            Timer.create(0, () -> arm.setWristPosition(.11), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(0, () -> {arm.setGrabberTarget(-125);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> {arm.setWristSpeed(0.0125); arm.setArmSpeed(0.0075);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .02),
            Timer.create(500, () -> arm.setGrabberTarget(arm.grabberLimit)),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> {arm.setArmPosition(Arm.ARM_MAX); arm.setWristSpeed(0.005);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> {ramp.setDefault(); arm.setArmSpeed(0.003);}, () -> arm.getArmPosition() > .73),
            // Timer.create(500, () -> {}),
            Timer.create(0, () -> arm.setWristPosition(0.11), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(0, () -> arm.setGrabberTarget(-1297)),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
        );
    }

}