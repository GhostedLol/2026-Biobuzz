package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class ShooterTest extends LinearOpMode {
    private DcMotor shooterMotor;

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            shooterMotor.setPower(1);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("shooterPower", shooterMotor.getPower());

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
