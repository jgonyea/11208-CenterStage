package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class TripleSwitch {
    DigitalChannel switchI;
    DigitalChannel switchII;

    public enum switchState {
        UP, MIDDLE, DOWN
    }

    public TripleSwitch(DigitalChannel switchI, DigitalChannel switchII) {
        this.switchI = switchI;
        this.switchII = switchII;
        switchI.setMode(DigitalChannel.Mode.INPUT);
        switchII.setMode(DigitalChannel.Mode.INPUT);
    }

    public switchState getState() {
        if (switchI.getState() == switchII.getState()) {
            return switchState.MIDDLE;
        } else {
            return !switchI.getState() ?
                    switchState.UP : switchState.DOWN;
        }
    }
}
