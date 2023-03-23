package frc.robot.sequence;

public class AutoSequence extends Sequence {
    
    public AutoSequence(Swerve swerve, Arm arm, Ramp ramp) {
        DunkSequence dunk = new DunkSequence(arm, ramp);
        ArmHomeSequence home = new ArmHomeSequence(arm, ramp);
        setSequence(
            Timer.create(0, () -> dunk.run()),
            Timer.create(0, () -> home.run(), () -> dunk.isComplete())
            //probably april tag home somewhere or another
            //dead reckon backwards
            //if we're really ambitious, use the gyro to balance but i dont think so
        );
    }

    //for rotation fix thing, get anglediff for new rotation from old rotation and just add that i think it worky

}
