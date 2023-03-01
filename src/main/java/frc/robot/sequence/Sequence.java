package frc.robot.sequence;

public class Sequence {
    private Timer[] timers;
    private boolean complete = true;

    public Sequence(Timer... timers) {
        this.timers = new Timer[timers.length + 1];
        this.timers[timers.length] = Timer.create(1000, () -> complete = true);
        for(int i = timers.length - 1; i >= 0; i--) {
            this.timers[i] = timers[i];
            this.timers[i].setNext(this.timers[i + 1]);
        }
    }

    // public Sequence(String script, Robot robot) {
    //     String[] coms = script.toUpperCase().split(";");
    //     this.timers = new Timer[coms.length - 1];
    //     for(int i = 0; i < coms.length - 1; i++) {
    //         String[] parts = coms[i].split(" ");
    //         // int delay = 0;
    //         // try {

    //         // }
    //         switch(coms[i]) {
    //         default:
    //             SmartDashboard.putString("Script Error", "Unknown command line " + (i + 1));
    //             break;
    //         case "ELH":
    //             this.timers[i] = Timer.create(1000);
    //         }
    //     }
    // }

    public  Sequence(int num_of_seq) {
        this.timers = new Timer[num_of_seq + 1];
        this.timers[num_of_seq] = Timer.create(1000, () -> complete = true);
    }



    protected void setSequence(Timer...timers) {
        this.timers = new Timer[timers.length + 1];
        this.timers[timers.length] = Timer.create(1000, () -> complete = true);
        for(int i = timers.length - 1; i >= 0; i--) {
            this.timers[i] = timers[i];
            this.timers[i].setNext(this.timers[i + 1]);
        }
    }

    protected void AddToSequence(int index, Timer New_Timer) {

        this.timers[index] = New_Timer;
        for(int i = index; i >= 0; i--) {
            this.timers[i].setNext(this.timers[i + 1]);
        }
    }

    public void reset() {
        complete = true;
    }

    public boolean run() {
        if(!complete) {
            return false;
        }
        complete = false;
        Timer.addTimer(timers[0]);
        return true;
    }

    public boolean isComplete() {
        return complete;
    }
    
}
