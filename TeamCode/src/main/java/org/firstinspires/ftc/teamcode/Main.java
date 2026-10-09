package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

/**
 * Do not add anything to this class. All subsystems, bindings and periodic logic live in
 * {@link Robot}.
 */
@TeleOp
public class Main extends CommandOpMode {
    @Override
    public void initialize() {
        new Robot(hardwareMap, gamepad1, telemetry);
    }
}
