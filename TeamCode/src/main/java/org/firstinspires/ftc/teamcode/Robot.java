package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    public final Drive drive;
    public final Intake intake;
    public final Indexer indexer;
    public final Shooter shooter;

    private final FtcDashboard dashboard;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        dashboard = FtcDashboard.getInstance();
    }

    public void update() {
        intake.update();

        TelemetryPacket packet = new TelemetryPacket();
        drive.addTelemetry(packet);
        intake.addTelemetry(packet);
        indexer.addTelemetry(packet);
        shooter.addTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);
    }
}
