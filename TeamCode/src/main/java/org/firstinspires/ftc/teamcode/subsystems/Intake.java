package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private static final double POWER = 0.5;

    private final DcMotor intakeMotor;

    private boolean running = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    public void toggle() {
        running = !running;
    }

    public void update() {
        if (running)
            intakeMotor.setPower(POWER);
        else
            intakeMotor.setPower(0);
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("intakePower", intakeMotor.getPower());
    }
}
