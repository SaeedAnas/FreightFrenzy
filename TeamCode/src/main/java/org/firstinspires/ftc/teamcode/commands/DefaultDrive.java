package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final Drive m_drive;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_rx;

    public DefaultDrive(Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rx) {
        m_drive = drive;
        m_x = x;
        m_y = y;
        m_rx = rx;
    }

    @Override
    public void execute() {
        m_drive.drive(m_x.getAsDouble(), m_y.getAsDouble(), m_rx.getAsDouble());
    }
}
