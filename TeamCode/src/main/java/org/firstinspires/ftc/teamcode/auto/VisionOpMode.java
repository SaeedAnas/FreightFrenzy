package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.ColorFilterPipeline;

@Config
@Autonomous(group = "blue")
public class VisionOpMode extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.stream();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double pos = ColorFilterPipeline.xPos;

            if (pos < 250) {
                telemetry.addData("Level", "BOTTOM");
            } else if (pos < 550) {
                telemetry.addData("Level", "MIDDLE");
            } else {
                telemetry.addData("Level", "TOP");
            }

//            telemetry.addData("xPos", ColorFilterPipeline.xPos);
            telemetry.update();

        }
    }

}

