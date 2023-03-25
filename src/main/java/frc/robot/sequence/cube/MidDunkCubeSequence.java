package frc.robot.sequence.cube;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class MidDunkCubeSequence extends Sequence {
    public MidDunkCubeSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.004); arm.setArmSpeed(0.004);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.569)),
            Timer.create(0, () -> {}, () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(1000, () -> {arm.setWristPosition(.117);}),
            Timer.create(500, () -> {}),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
        );
    }

}