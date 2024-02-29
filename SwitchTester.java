package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Config Switch Tester")
public class SwitchTester extends OpMode {
    DigitalChannel leftSwitch;
    DigitalChannel centerSwitch;
    DigitalChannel rightSwitchI;
    DigitalChannel rightSwitchII;
    TripleSwitch rightSwitch;

    @Override
    public void init() {
        leftSwitch = hardwareMap.get(DigitalChannel.class, "switch3");
        centerSwitch = hardwareMap.get(DigitalChannel.class, "switch2");
        rightSwitchI = hardwareMap.get(DigitalChannel.class, "parkSwitchI");
        rightSwitchII = hardwareMap.get(DigitalChannel.class, "parkSwitchII");
        rightSwitch = new TripleSwitch(rightSwitchI, rightSwitchII);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Switch", !leftSwitch.getState() ? "DOWN" : "UP");
        telemetry.addData("Center Switch", !centerSwitch.getState() ? "DOWN" : "UP");
        telemetry.addData("Right Switch", rightSwitch.getState().name());
    }
}
