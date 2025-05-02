package pedroPathing.gamepad;

public class ButtonToggle {
    private boolean lastState = false;
    private boolean toggled = false;

    public void update(boolean currentState) {
        if (justPressed(currentState)) {
            toggled = !toggled;
        }
        lastState = currentState;
    }

    public boolean justPressed(boolean currentState) {
        return currentState && !lastState;
    }

    public boolean justReleased(boolean currentState) {
        return !currentState && lastState;
    }

    public boolean isToggled() {
        return toggled;
    }
}
