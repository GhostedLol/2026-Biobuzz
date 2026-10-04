package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter extends SubsystemBase {
    private static final double RPM = 400;
    // 1 RPM = 6 degrees per second
    private static final double DEGREES_PER_SECOND_PER_RPM = 6.0;

    private final DcMotorEx shooterMotor;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public Command stop() {
        return runOnce(() -> shooterMotor.setPower(0));
    }

    public Command setRPM(double rpm) {
        return run(() -> shooterMotor.setVelocity(rpm * DEGREES_PER_SECOND_PER_RPM, AngleUnit.DEGREES));
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("shooterPower", shooterMotor.getPower());
        packet.put("shooterRPM", shooterMotor.getVelocity());
    }
}
