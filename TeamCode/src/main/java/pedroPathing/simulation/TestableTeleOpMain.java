package pedroPathing.simulation;

import pedroPathing.examples.TeleOpMain;

public class TestableTeleOpMain extends TeleOpMain {
    private boolean active = false;

    public void setActive(boolean active) {
        this.active = active;
    }

//    @Override
//    public boolean opModeIsActive() {
//        return active;
//    }

    @Override
    public void waitForStart() {
        // Simulate immediate start
    }
}