package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Dumpy;

@Config
@Autonomous(group = "drive")
public class DumpyTuner extends LinearOpMode {
    Dumpy dumpy;

    public static double topPos = 0;
    public static double bottomPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        dumpy = new Dumpy(hardwareMap, robot);

        waitForStart();

        while(opModeIsActive()) {
            if (isStopRequested()) return;

            dumpy.toPos(topPos, bottomPos);
        }
    }
}
