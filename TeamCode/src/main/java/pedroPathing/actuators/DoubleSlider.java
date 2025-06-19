package pedroPathing.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleSlider {
    Slider slider1, slider2;
    public DoubleSlider(String slider1Name, String slider2Name, HardwareMap hwmap) {
        slider1 = new Slider(slider1Name, hwmap, DcMotor.Direction.FORWARD, true);
        slider2 = new Slider(slider2Name, hwmap, DcMotor.Direction.REVERSE, true);
    }
    public void update() {
        slider1.update();
        slider2.update();
    }
    public void relax() {
        slider1.relax();
        slider2.relax();
    }
    public void run() {
        slider1.run();
        slider2.run();
    }
    public void setPositionTicks(int ticks) {
        slider1.setPositionTicks(ticks);
        slider2.setPositionTicks(ticks);
    }
    public boolean isRelaxed() {
        return slider1.isRelaxed();
    }
    public int getCurrentPositionTicks() {
        return slider1.getCurrentPositionTicks();
    }
    public void changeState() {
        slider1.changeState();
        slider2.changeState();
    }
}
