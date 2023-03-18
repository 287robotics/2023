package frc.robot.sequence.swerve;

import frc.robot.Swerve;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class LeftConeSequence extends Sequence {

    private final static double [] coneX = {6.27, 6.35, 6.3, 0, 0, -6.38};
    private final static double [] coneY = {-2.45, -0.77, 0.88, 0, 0, -0.059};

    private int currentTargetID = 0;

    public LeftConeSequence(Swerve swerve) {
        setSequence(
            Timer.create(0, () -> {swerve.calibrateBotPosition(); swerve.setBotTarget(coneX[currentTargetID - 1], coneY[currentTargetID - 1]); swerve.setHomingBotEnabled(true);}, () -> swerve.distanceFromRotationTarget() < 0.1),
            Timer.create(0, () -> {}, () -> swerve.distanceSquaredFromBotTarget() < 0.0025),
            Timer.create(1000, () -> swerve.setHomingBotEnabled(false)));
    }

    public void setTargetID(int id) {
        this.currentTargetID = id;
    }
    
}
