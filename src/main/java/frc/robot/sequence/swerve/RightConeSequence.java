package frc.robot.sequence.swerve;

import frc.robot.Swerve;
import frc.robot.sequence.Sequence;
import frc.robot.sequence.Timer;

public class RightConeSequence extends Sequence {

    private final static double [] coneX = {6.27, 6.35, 6.35, 0, 0, -6.37};
    private final static double [] coneY = {-3.53, -1.94, -0.175, 0, 0, 1.03};

    private int currentTargetID = 0;

    public RightConeSequence(Swerve swerve) {
        setSequence(
            Timer.create(0, () -> {swerve.setRotationTarget(Math.PI / 4);}),
            Timer.create(0, () -> {swerve.calibrateBotPosition(); swerve.setBotTarget(coneX[currentTargetID - 1], coneY[currentTargetID - 1]); swerve.setHomingBotEnabled(true);}, () -> swerve.distanceFromRotationTarget() < 0.1),
            Timer.create(0, () -> {}, () -> swerve.distanceSquaredFromBotTarget() < 0.0025),
            Timer.create(500, () -> {swerve.setRotationTarget(0); swerve.setHomingBotEnabled(false);}));
    }

    public void setTargetID(int id) {
        this.currentTargetID = id;
    }
    
}
