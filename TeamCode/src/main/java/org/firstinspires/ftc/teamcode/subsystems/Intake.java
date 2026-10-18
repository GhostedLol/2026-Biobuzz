package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.command.Commands.run;
import static com.seattlesolvers.solverslib.command.Commands.runEnd;
import static com.seattlesolvers.solverslib.command.Commands.runOnce;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;

public class Intake {

    private final DcMotor intakeMotor;

    private double currentPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public Command run(double power) {
        return runOnce(() -> setPower(power));
    }

    public Command stop() {return runOnce(() -> setPower(0));}

    public Command toggle() {
        return runOnce(() -> setPower(currentPower > 0 ? 0 : 1));
    }

    public Command reverse() {
        return runOnce(() -> setPower(currentPower < 0 ? 0 : -1));
    }

    private void setPower(double power) {
        intakeMotor.setPower(power);
        currentPower = power;
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("intakePower", intakeMotor.getPower());
    }
}
