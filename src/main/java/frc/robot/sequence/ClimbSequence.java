package frc.robot.sequence;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class ClimbSequence extends Sequence {

    private static final double MID_STOP = 0.55;

    public ClimbSequence(Robot robot) {
        setSequence(
            // Timer.create(0, () -> robot.intakeSolenoid.set(Value.kForward)), //put intake arm up
            // Timer.create(0, () -> robot.retract()), //pivot elevator out
            // Timer.create(0, () -> robot.armPosition(0.85)), //arm position to 85%
            // Timer.create(0, () -> robot.extend(), () -> robot.elevator.getSelectedSensorPosition() >= (int) (8192 * 55 * 0.8)), //pivot elevator in when elevator reaches 80% height
            // Timer.create(500, () -> robot.armIn()), // brings arm in after a second
            // Timer.create(0, () -> robot.armPosition(0.5), () -> robot.elevator.getSelectedSensorPosition() < 9192), //when arm gets to the bottom, extend elevator to half
            // Timer.create(0, () -> robot.retract(), () -> robot.elevator.getSelectedSensorPosition() >= (int) (8192 * 55 * 0.47)),
            // Timer.create(1000, () -> robot.armPosition(1.0)),
            // Timer.create(0, () -> robot.extend(), () -> robot.elevator.getSelectedSensorPosition() >= (int) (8192 * 55 * 0.99)),
            // Timer.create(1000, () -> robot.armPosition(MID_STOP)), //how far it goes up before testing limit switch
            // Timer.create(0, () -> robot.armIn(), () -> !robot.pivotLimit.get()),
            // Timer.create(0, () -> robot.armPosition(0.5), () -> robot.elevator.getSelectedSensorPosition() < 9192),
            // Timer.create(1000, () -> robot.retract()),
            // Timer.create(1000, () -> robot.armPosition(1.0)),
            // Timer.create(0, () -> robot.extend(), () -> robot.elevator.getSelectedSensorPosition() >= (int) (8192 * 55 * 0.99)),
            // Timer.create(2000, () -> robot.armPosition(MID_STOP))
            
            // Timer.create(0, () -> robot.armIn(), () -> !robot.pivotLimit.get()),
            // Timer.create(0, () -> robot.armPosition(0.5), () -> robot.elevator.getSelectedSensorPosition() < 9192)
        );
    }
    
}
