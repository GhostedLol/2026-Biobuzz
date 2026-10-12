package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // goBILDA 5000 Series 12VDC motor (5000-0002-4008), 6000 RPM, no gearbox: 28 encoder ticks per revolution
    private static final double TICKS_PER_REV = 28.0;
    private static final double TICKS_PER_SECOND_PER_RPM = TICKS_PER_REV / 60.0;

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;

    public Shooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void stopMotors() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public Command stop() {
        return runOnce(this::stopMotors);
    }

    /** Holds the given speed until the command is cancelled, then stops the motors. */
    public Command setRPM(double rpm) {
        return runEnd(() -> {
            leftShooter.setVelocity(rpm * TICKS_PER_SECOND_PER_RPM);
            rightShooter.setVelocity(rpm * TICKS_PER_SECOND_PER_RPM);
        }, this::stopMotors);
    }

    public Command setPower(double power) {
        return runEnd(() -> {
            leftShooter.setPower(power);
            rightShooter.setPower(power);
        }, this::stopMotors);
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("shooterPower1", leftShooter.getPower());
        packet.put("shooterPower2", rightShooter.getPower());
        packet.put("shooterRPM1", leftShooter.getVelocity() / TICKS_PER_SECOND_PER_RPM);
        packet.put("shooterRPM2", rightShooter.getVelocity() / TICKS_PER_SECOND_PER_RPM);
    }
}
