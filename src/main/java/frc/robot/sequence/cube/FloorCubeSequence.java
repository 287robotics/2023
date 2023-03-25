package frc.robot.sequence.cube;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class FloorCubeSequence extends Sequence {
    public FloorCubeSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.7052)),
            Timer.create(0, () -> {arm.setGrabberTarget(-125);}, () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(500, () -> {}),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
        );
    }

}