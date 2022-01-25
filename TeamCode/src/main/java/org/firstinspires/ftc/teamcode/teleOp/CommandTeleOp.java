package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.Webcam;

@TeleOp(name="Command TeleOp", group="TeleOp")
public class CommandTeleOp extends CommandOpMode {

    private GamepadEx driveOp;
    private GamepadEx toolOp;
    private Drive drive;
    private Arm arm;
    private Intake intake;
    private Webcam webcam;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        webcam = new Webcam(hardwareMap);

        driveOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        StartEndCommand intakeOn = new StartEndCommand(intake::intake, intake::stop, intake);

        StartEndCommand outtakeOn = new StartEndCommand(intake::outtake, intake::stop, intake);

        StartEndCommand topLevel = new StartEndCommand(() -> arm.setState(Arm.State.TOP), arm::off, arm);
        StartEndCommand intakeLevel = new StartEndCommand(() -> arm.setState(Arm.State.INTAKE), arm::off, arm);

        DefaultDrive driveCommand = new DefaultDrive(drive, () -> driveOp.getLeftX(), () -> driveOp.getLeftY(), () -> driveOp.getRightX());

        driveOp.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(intakeOn);

        driveOp.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(outtakeOn);

        driveOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(topLevel);

        driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(intakeLevel);

        register(drive, arm);
        drive.setDefaultCommand(driveCommand);
        arm.setDefaultCommand(new RunCommand(arm::toPos));

        webcam.aprilTagDetection().start();
    }
}
