package frc.robot.sequence;

import frc.robot.Arm;
import frc.robot.Ramp;
import frc.robot.Swerve;

public class AutoSequence extends Sequence {
    
    public AutoSequence(Swerve swerve, Arm arm, Ramp ramp) {        
        setSequence(
            Timer.create(0, () -> swerve.balancing = true),
            //dunk
            Timer.create(0, () -> {arm.setWristSpeed(.006); arm.setArmSpeed(0.007);}),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> ramp.setDown(), () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setArmPosition(0.383)),
            Timer.create(0, () -> {arm.setArmSpeed(.002); arm.setWristSpeed(0.0035); arm.setWristPosition(-0.215); arm.setArmPosition(.43);}, () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            //not dunk
            Timer.create(250, () -> arm.homeGrabber(), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01 && Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> arm.setWristSpeed(.006)),
            Timer.create(0, () -> arm.setWristPosition(.2), () -> arm.isHomed()),
            Timer.create(0, () -> {swerve.setReckoningTarget(80, -Math.PI / 2); swerve.setReckoningEnabled(true);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .03),
            //homing
            Timer.create(0, () -> {arm.setWristSpeed(0.0125); arm.setArmSpeed(0.0075);}),
            Timer.create(0, () -> arm.setGrabberTarget(arm.grabberLimit)),
            Timer.create(0, () -> arm.setWristPosition(0.2)),
            Timer.create(0, () -> {arm.setArmPosition(Arm.ARM_MAX); arm.setWristSpeed(0.005);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .01),
            Timer.create(0, () -> {ramp.setDefault(); arm.setArmSpeed(0.003);}, () -> arm.getArmPosition() > .73),
            Timer.create(0, () -> arm.setWristPosition(0.148), () -> Math.abs(arm.getArmPosition() - arm.getArmTarget()) < .01),
            Timer.create(0, () -> arm.setGrabberTarget(-200)),
            Timer.create(0, () -> {arm.setWristSpeed(0.002); arm.setArmSpeed(0.002);}, () -> Math.abs(arm.getWristPosition() - arm.getWristTarget()) < .005)
            //not homing
            // Timer.create(0, () -> {swerve.setStopOnDeviation(true); swerve.setStopOnDeviationValue(swerve.getPigeon().getRoll());}, () -> !(swerve.getReckoning()))
            //probably april tag home somewhere or another
            //dead reckon backward
            //if we're really ambitious, use the gyro to balance but i dont think so
        );
    }

    //for rotation fix thing, get anglediff for new rotation from old rotation and just add that i think it worky

}
