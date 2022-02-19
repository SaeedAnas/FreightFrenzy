package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@Autonomous(group = "drive")
public class IntakeTuner extends LinearOpMode {
    Intake intake;

    // 0 is intake position
    public static double leftPower = 0;
    public static double rightPower = 0;

    // PivotServoLeft = 0 // Second Arm Left "secondArmL"
    // PivotServoRight = 1 // Second Arm Right "secondArmR"
    // ArmServoLeft = 2 // ArmDumpy Left "leftArmDumpy"
    // ArmServoRight = 3 // ArmDumpy Right "rightArmDumpy"
    // Dumper = 5 // Dumpy "wall"

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap, null);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            intake.intakeLeft.setPower(leftPower);
            intake.intakeRight.setPower(rightPower);


        }


    }
}
