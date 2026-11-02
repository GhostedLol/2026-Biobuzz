package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class Autos extends CommandOpMode {
    private static final double DRIVE_POWER = 0.4;
    private static final double Sh = 1300;
    private static final double FEED_POWER = 0.5;

    private static final long SPIN_UP_MILLIS = 1000;
    private static final long SHOOT_MILLIS = 4000;

    // OpModes need a no-arg constructor, so everything is created in initialize()
    private Drive drive;
    private Indexer indexer;
    private Shooter shooter;
    private Intake intake;
    private Follower follower;

//    Poses
    private final PoseFactory p = PoseFactory.degrees();

//    public Command testingSequence() {
//        return Commands.parallel(
//                drive.bl(1).withTimeout(1),
//                drive.bl(1).withTimeout(1),
//                drive.bl(1).withTimeout(1),
//                drive.bl(1).withTimeout(1),
//                intake.run(1),
//                Commands.waitMillis(2000),
//                shooter.shoot(indexer, intake).withTimeout(3000));
//    }

    // Pedro poses are in inches and radians, x is forward and y is left of where the robot starts.
    public Command pathAuto() {
        Pose start = new Pose(136.2, 15, Math.toRadians(180));
        Pose point2 = new Pose(56, 15, Math.toRadians(180));
        Pose point3 = new Pose(56, 15, Math.toRadians(90));
        Pose shoot = new Pose(56, 7.8, Math.toRadians(90));

        Path part1 = line(start, point2).linear(start, point2);
        // Start from point2 so the heading turns 180 -> 90 along the path instead of jumping
        Path part2 = line(point2, shoot).linear(point2, shoot);

        return Commands.sequence(
                drive.runOnce(() -> follower.setPose(start)),
                Commands.sequence(
                    followPath(part1, point2).alongWith(intake.run(1)),
                    followPath(part2, shoot),
                    shooter.run(indexer, intake)));
    }

    public Command turn90() {
        Pose start = new Pose(136.2, 15, Math.toRadians(180));
        Pose point2 = new Pose(133, 15, Math.toRadians(90));

        Path path = line(start, point2).linear(start, point2);
        return Commands.sequence(
                drive.runOnce(() -> follower.setPose(start)),
                followPath(path, point2));
    }

    private Command followPath(Path path, Pose finishPos) {
        return new FunctionalCommand(
                () -> follower.follow(path),
                () -> {},
                interrupted -> {
                    if (interrupted) follower.stop();
                },
                () -> Math.abs(finishPos.x() - drive.getPos().xMm) < 0.5 & Math.abs(finishPos.y() - drive.getPos().yMm) < 0.5 & Math.abs(finishPos.heading() - drive.getPos().headingRad) < 0.15,
                drive);
    }

    @Override
    public void initialize() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new Drive(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = Constants.create(hardwareMap);
        if (follower == null)
            throw new IllegalStateException("Constants.create() returned null, fill in pedro/Constants.java with your tuned values");

        schedule(new RunCommand(follower::update), pathAuto());

        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            drive.addTelemetry(packet);
            intake.addTelemetry(packet);
            indexer.addTelemetry(packet);
            shooter.addTelemetry(packet);
            packet.put("heading deg", Math.toDegrees(follower.pose().heading()));
            packet.put("heading vel deg/s", Math.toDegrees(follower.twist().omega));
            packet.put("pose", follower.pose().toString());
            packet.put("following", follower.following());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("heading deg", "%.1f", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("heading vel deg/s", "%.1f", Math.toDegrees(follower.twist().omega));
            telemetry.addData("pose", follower.pose());
            telemetry.addData("following", follower.following());
            telemetry.update();
        }));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
