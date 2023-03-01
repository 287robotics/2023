package frc.robot.sequence;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;

public abstract class Timer {

    // private static boolean tDown = false;
    // private static boolean tJDown = false;

    public static ArrayList<Timer> timers = new ArrayList<>();

    public static boolean paused = false;

    public static void clear() {
        timers.clear();
    }

    public static void addTimer(long delay, TimerAction action) {
        timers.add(new Timer(delay) {
            @Override
            public void action() {
                super.action();
                action.action();
            }
        });
    }

    public static Timer create(long delay, TimerAction action) {
        return new Timer(delay) {
            @Override
            public void action() {
                super.action();
                action.action();
            }
        };
    }

    public static Timer create(long delay, TimerAction action, TimerTest test) {
        return new Timer(delay) {
            @Override
            public void action() {
                super.action();
                action.action();
            }
            @Override
            protected boolean check() {
                if(test.check()) {
                    action();
                    return true;
                }
                return false;
            }

        };
    }

    public static void addTimer(Timer timer) {
        timers.add(timer);
        timer.start();
    }

    public static void setPaused(boolean paused) {
        Timer.paused = paused;
    }

    public static void checkAllTimers(XboxController controller) {
        // tJDown = controller.getLeftBumper() && !tDown;
        // tDown = controller.getLeftBumper();
        if(!paused) {
            for(int i = 0; i < timers.size(); i++) {
                if(timers.get(i).check()) {
                    timers.remove(i);
                    i--;
                }
            }
        }
        // if(tJDown) {
        //     timers.get(0).action();
        //     timers.remove(0);
        // }
    }

    private long time;
    public final long delay;
    private Timer next = null;

    public Timer(long delay) {
        this.delay = delay;
    }

    public void setNext(Timer t) {
        this.next = t;
    }

    private void start() {
        time = delay + System.currentTimeMillis();
    }

    public void action() {
        if(next != null) {
            Timer.addTimer(next);
        }
    };

    protected boolean check() {
        if(System.currentTimeMillis() > time) {
            action();
            return true;
        }
        return false;
    }
    
}