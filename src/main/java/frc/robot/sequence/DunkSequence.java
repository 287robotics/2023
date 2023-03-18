package frc.robot.sequence;

import frc.robot.Arm;
import frc.robot.Ramp;

public class DunkSequence extends Sequence {
    
    public DunkSequence(Arm arm, Ramp ramp) {
        setSequence(
            Timer.create(0, () -> {arm.setWristSpeed(0.007); arm.setArmSpeed(0.005);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.426)),
            Timer.create(0, () -> {arm.setWristSpeed(0.0035); arm.setWristPosition(-0.2231);}, () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(500, () -> {}),
            // Timer.create(0, () -> arm.setWristPosition(-0.2231), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .005),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
        );
    }

}
