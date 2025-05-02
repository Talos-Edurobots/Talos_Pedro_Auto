package pedroPathing.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadButtonHandler {
    public ButtonToggle a = new ButtonToggle();
    public ButtonToggle b = new ButtonToggle();
    public ButtonToggle x = new ButtonToggle();
    public ButtonToggle y = new ButtonToggle();
    public ButtonToggle dpad_up = new ButtonToggle();
    public ButtonToggle dpad_down = new ButtonToggle();
    public ButtonToggle dpad_left = new ButtonToggle();
    public ButtonToggle dpad_right = new ButtonToggle();
    public ButtonToggle left_bumper = new ButtonToggle();
    public ButtonToggle right_bumper = new ButtonToggle();

    // You can add triggers or joysticks too, if needed

    public void update(Gamepad gamepad) {
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        dpad_up.update(gamepad.dpad_up);
        dpad_down.update(gamepad.dpad_down);
        dpad_left.update(gamepad.dpad_left);
        dpad_right.update(gamepad.dpad_right);
        left_bumper.update(gamepad.left_bumper);
        right_bumper.update(gamepad.right_bumper);
    }
}
