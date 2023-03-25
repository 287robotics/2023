package frc.robot.sequence.cube;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class DunkCubeSequence extends Sequence {

    public DunkCubeSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.007); arm.setArmSpeed(0.006);}),
            Timer.create(0, () -> arm.setWristPosition(0)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0)),
            Timer.create(0, () -> {arm.setWristSpeed(0.0035); arm.setWristPosition(0);}, () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(0, () -> {arm.setGrabberTarget(-1000);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);})
        );
    }

}
