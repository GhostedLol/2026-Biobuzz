package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Autos;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class ExampleAuto extends CommandOpMode {
    private Autos autos;

    @Override
    public void initialize() {
        autos = new Autos(new Drive(hardwareMap), new Indexer(hardwareMap), new Shooter(hardwareMap), new Intake(hardwareMap));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Scheduled here rather than in initialize() so the routine's timers start when the match
    // does, not while the robot is still waiting for start.
    @Override
    public void preRun() {
        schedule(autos.exampleAuto());
    }
}
