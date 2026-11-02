package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

public class Shooter extends SubsystemBase {

    // goBILDA 5000 Series 12VDC motor (5000-0002-4008), 6000 RPM, no gearbox: 28 encoder ticks per revolution
    private static final double TICKS_PER_REV = 28.0;
    // Feed once within this many RPM of the target; the velocity PID tends to settle just under it
    private static final double RPS_TOLERANCE = 1;
    public final double SHOOTER_RPS = 57;
    private static final double BREAKAWAY_RPS = 1.66;
    private static final double RAMP_VOLTS_PER_SECOND = 0.25;

    private double appliedVoltage = 0;
    private double breakawayVoltage = 0;
    
    private final VoltageSensor batteryVoltage;
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    PIDFController pidf = new PIDFController(2, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.4, 0.135, 0.119);

    public Shooter(HardwareMap hardwareMap) {
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public Command stop() {
        return runOnce(() -> {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        });
    }
    public Command run() {
        pidf.reset();
        return runEnd(() -> {
            double pid_output =
                    pidf.calculate(getRPS(), SHOOTER_RPS);
            double feed_output =
                    feedforward.calculate(SHOOTER_RPS, 0);

            leftShooter.setPower((pid_output+feed_output)/batteryVoltage.getVoltage());
            rightShooter.setPower((pid_output+feed_output)/batteryVoltage.getVoltage());
        }, this::stop);
    }
    public Command shoot(Indexer indexer, Intake intake) {
        return runEnd(() -> {
            Commands.parallel(
                run(),
                intake.run(0.5),
                Commands.waitUntil(this::atRPS).andThen(indexer.feed(1)));
        }, this::stop);
    }


    public boolean atRPS() {
        return
                getRPS() >= (SHOOTER_RPS - RPS_TOLERANCE) &
                getRPS() <= (SHOOTER_RPS + RPS_TOLERANCE);
    }

    public double getRPS() {
        double avgVel = (((leftShooter.getVelocity() + rightShooter.getVelocity()) / 2) / TICKS_PER_REV);
        return avgVel;
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("shooterPower1", leftShooter.getPower());
        packet.put("shooterPower2", rightShooter.getPower());
        packet.put("shooterRPS1", leftShooter.getVelocity() / TICKS_PER_REV);
        packet.put("shooterRPS2", rightShooter.getVelocity() / TICKS_PER_REV);
        packet.put("leftTPS", leftShooter.getVelocity());
        packet.put("rightTPS", rightShooter.getVelocity());
        packet.put("shooterAppliedVoltage", appliedVoltage);
        packet.put("shooterBreakawayVoltage", breakawayVoltage);
    }
}
