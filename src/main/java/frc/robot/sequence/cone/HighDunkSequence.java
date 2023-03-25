package frc.robot.sequence.cone;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class HighDunkSequence extends Sequence {
    public HighDunkSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.007); arm.setArmSpeed(0.0075);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.398)),
            Timer.create(0, () -> {arm.setWristSpeed(0.0045); arm.setWristPosition(-0.291);}, () -> Math.abs(arm.getArmTarget() - arm.getArmPosition()) < .01),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .09)
        );
    }
}
