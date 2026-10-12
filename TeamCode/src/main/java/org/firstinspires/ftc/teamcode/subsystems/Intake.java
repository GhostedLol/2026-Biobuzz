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

    private boolean running = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void toggle() {
        running = !running;
    }

    public void update() {
        if (running)
            intakeMotor.setPower(1);
        else
            intakeMotor.setPower(0);
    }

    public Command run(double power) {
        return runOnce(() -> intakeMotor.setPower(power));
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("intakePower", intakeMotor.getPower());
    }
}
