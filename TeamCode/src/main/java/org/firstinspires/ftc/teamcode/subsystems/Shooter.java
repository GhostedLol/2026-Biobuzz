package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final double RPM = 400;
    // goBILDA 5203 Yellow Jacket 1620 RPM (3.7:1): 103.8 encoder ticks per output shaft revolution
    private static final double TICKS_PER_REV = 103.8;
    private static final double TICKS_PER_SECOND_PER_RPM = TICKS_PER_REV / 60.0;

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void stopMotors() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    public Command stop() {
        return runOnce(this::stopMotors);
    }

    /** Holds the given speed until the command is cancelled, then stops the motors. */
    public Command setRPM(double rpm) {
        return runEnd(() -> {
            shooterMotor1.setVelocity(rpm * TICKS_PER_SECOND_PER_RPM);
            shooterMotor2.setVelocity(rpm * TICKS_PER_SECOND_PER_RPM);
        }, this::stopMotors);
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("shooterPower1", shooterMotor1.getPower());
        packet.put("shooterPower2", shooterMotor2.getPower());
        packet.put("shooterRPM1", shooterMotor1.getVelocity() / TICKS_PER_SECOND_PER_RPM);
        packet.put("shooterRPM2", shooterMotor2.getVelocity() / TICKS_PER_SECOND_PER_RPM);
    }
}
