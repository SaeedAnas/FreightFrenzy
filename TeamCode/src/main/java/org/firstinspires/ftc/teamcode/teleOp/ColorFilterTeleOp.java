package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ColorFilterPipeline;
import org.firstinspires.ftc.teamcode.vision.TestPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@TeleOp
public class ColorFilterTeleOp extends LinearOpMode {
    OpenCvCamera camera;

    int left_x_val = 100; //NEEDS TUNING!
    int right_x_val = 200; //NEEDS TUNING!


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        ColorFilterPipeline limePipeline = new ColorFilterPipeline();
        int object_position = limePipeline.getPos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.setMsTransmissionInterval(50);

        telemetry.addData("XPosition of Object", object_position);
        telemetry.addLine(String.format("Object Position (actual value): %d" , object_position));

        int positionVar = 3;
        //left side: positionVar = 1
        //center: positionVar = 2
        //right side: positionVar = 3

        if (object_position < left_x_val)
            positionVar = 1;
        else if (object_position > right_x_val)
            positionVar = 3;
        else
            positionVar = 2;

        telemetry.addData("PositionVar", positionVar);
        telemetry.addLine(String.format("Position (left/middle/right): %d" , positionVar));

        telemetry.update();
    }
}
