package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name="camera test")
public class CameraTest extends OpMode {
    private WebcamName logi1;
    private TfodProcessor tfodProcessor;
    private VisionPortal visionPortal;
    private int loopCounter;

    @Override
    public void init() {
        logi1 = hardwareMap.get(WebcamName.class, "logi1");
        tfodProcessor = TfodProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(logi1, tfodProcessor);
    }

    @Override
    public void loop() {
        List<Recognition> recognitions = tfodProcessor.getRecognitions();

        for (Recognition recognition : recognitions) {
            String label = recognition.getLabel();
            float confidence = recognition.getConfidence();

            telemetry.addLine("Recognized object" + label);
            telemetry.addLine("Confidence" + confidence);
        }

        telemetry.addData("recognitions.size()", recognitions.size());
        telemetry.addData("Loop", loopCounter++);
    }
}
