package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.command.Commands.runOnce;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

public class Flower {
    private final Servo flowerServo;

    public Flower(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
    }

    public Command reset() {return runOnce(() -> flowerServo.setPosition(0));}
    public Command run() {return runOnce(() -> flowerServo.setPosition(0.1));}

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("intakePower", flowerServo.getPosition());
    }
}
