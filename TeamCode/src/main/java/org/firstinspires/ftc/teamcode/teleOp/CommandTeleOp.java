package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class CommandTeleOp extends CommandOpMode {

    private GamepadEx gamepad;
    private Drive drive;
    private Arm arm;
    private Intake intake;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        StartEndCommand intakeOn = new StartEndCommand(intake::intake, intake::stop, intake);

        StartEndCommand outtakeOn = new StartEndCommand(intake::outtake, intake::stop, intake);

        StartEndCommand topLevel = new StartEndCommand(() -> arm.setState(Arm.State.TOP), arm::off, arm);
        StartEndCommand intakeLevel = new StartEndCommand(() -> arm.setState(Arm.State.INTAKE), arm::off, arm);

        DefaultDrive driveCommand = new DefaultDrive(drive, () -> gamepad.getLeftX(), () -> gamepad.getLeftY(), () -> gamepad.getRightX());

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(intakeOn);

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(outtakeOn);

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(topLevel);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(intakeLevel);

        register(drive, arm);
        drive.setDefaultCommand(driveCommand);
        arm.setDefaultCommand(new RunCommand(arm::toPos));
    }
}
