package frc.robot.sequence.cone;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class DunkSequence extends Sequence {
    
    public DunkSequence(Arm arm, Ramp ramp) {
        this(arm, ramp, .007, .0045, .0075);
    }
    public DunkSequence(Arm arm, Ramp ramp, double wristSpeed1, double wristSpeed2, double armSpeed) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(wristSpeed1); arm.setArmSpeed(armSpeed);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.383)),
            Timer.create(0, () -> {arm.setWristSpeed(wristSpeed2); arm.setWristPosition(-0.215);}, () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .09)
        );
    }

}
