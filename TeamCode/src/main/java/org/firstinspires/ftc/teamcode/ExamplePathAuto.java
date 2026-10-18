package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Autos;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class ExamplePathAuto extends CommandOpMode {
    private Follower follower;
    private Autos autos;

    @Override
    public void initialize() {
        Drive drive = new Drive(hardwareMap);

        // Created after Drive so Pedro's motor directions are the ones that apply.
        follower = Constants.create(hardwareMap);
        if (follower == null)
            throw new IllegalStateException("Pedro is not set up, Constants.create() returns null");

        autos = new Autos(drive, new Indexer(hardwareMap), new Shooter(hardwareMap), new Intake(hardwareMap), follower);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void preRun() {
        schedule(autos.pathAuto());
    }

    // The follower has to be updated every loop, including between paths, so it can track the
    // robot's position and hold the end of the last path.
    @Override
    public void run() {
        follower.update();
        super.run();
    }
}
