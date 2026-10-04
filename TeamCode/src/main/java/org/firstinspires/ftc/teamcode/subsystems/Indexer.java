package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final DcMotor indexerMotor;

    public Indexer(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.get(DcMotor.class, "indexerMotor");
    }

    public Command run(double power) {
        return runOnce(() -> indexerMotor.setPower(power));
    }

    public Command stop(){
        return runOnce(() -> indexerMotor.setPower(0));
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("indexerPower", indexerMotor.getPower());
    }
}
