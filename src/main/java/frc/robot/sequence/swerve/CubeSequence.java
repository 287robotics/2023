package frc.robot.sequence.swerve;

import frc.robot.Swerve;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class CubeSequence extends Sequence {

    private final static double [] cubeX = {6.33, 6.34, 6.3, 0, 0, -6.3};
    private final static double [] cubeY = {-2.98, -1.30, 0.35, 0, 0, 0.47};

    private int currentTargetID = 1;

    public CubeSequence(Swerve swerve) {
        setSequence(
            Timer.create(0, () -> {swerve.calibrateBotPosition(); swerve.setBotTarget(cubeX[currentTargetID - 1], cubeY[currentTargetID - 1]); swerve.setHomingBotEnabled(true);}, () -> swerve.distanceFromRotationTarget() < 0.1),
            Timer.create(0, () -> {}, () -> swerve.distanceSquaredFromBotTarget() < 0.0025),
            Timer.create(1000, () -> swerve.setHomingBotEnabled(false)));
    }

    public void setTargetID(int id) {
        this.currentTargetID = id;
    }
    
}
