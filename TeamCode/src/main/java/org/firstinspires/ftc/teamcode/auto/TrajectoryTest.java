package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group="test")
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
            .strafeRight(10)
            .forward(5)
            .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}
