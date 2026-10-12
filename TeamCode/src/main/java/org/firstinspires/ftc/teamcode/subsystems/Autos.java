package org.firstinspires.ftc.teamcode.subsystems;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

public class Autos {
    private static final double DRIVE_POWER = 0.4;
    private static final double SHOOTER_RPM = 1300;
    private static final double FEED_POWER = 0.5;

    private static final long SPIN_UP_MILLIS = 1000;
    private static final long SHOOT_MILLIS = 4000;

    private final Drive drive;
    private final Indexer indexer;
    private final Shooter shooter;
    private final Intake intake;
    private final Follower follower;

    public Autos(Drive drive, Indexer indexer, Shooter shooter, Intake intake) {
        this(drive, indexer, shooter, intake, null);
    }

    public Autos(Drive drive, Indexer indexer, Shooter shooter, Intake intake, Follower follower) {
        this.follower = follower;
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        this.intake = intake;
    }

    public Command exampleAuto() {
        return Commands.sequence(
                driveFor(DRIVE_POWER, 0, 0, 1000),
                Commands.waitMillis(500),
                shoot(SHOOT_MILLIS),
                driveFor(0, DRIVE_POWER, 0, 1000));
    }

    public Command testingSequence() {
        return Commands.parallel(
//                driveFor(0.5, 0, 0, 200
//                0),
//                Commands.waitMillis(2000),
                shooter.setRPM(4000),
                intake.run(1),
                indexer.run(1)
        );
    }

    // Pedro poses are in inches and radians, x is forward and y is left of where the robot starts.
    public Command pathAuto() {
        Pose start = new Pose(0, 0, 0);
        Pose shootPose = new Pose(24, 0, 0);
        Pose park = new Pose(48, 24, 0);

        Path toShoot = line(start, shootPose).constant(0);
        Path toPark = curve(shootPose, new Pose(48, 0), park).tangent();

        return Commands.sequence(
                drive.runOnce(() -> follower.setPose(start)),
                followPath(toShoot),
                shoot(SHOOT_MILLIS),
                followPath(toPark));
    }

    // Only starts the path and waits for it, the OpMode has to call follower.update() every loop.
    private Command followPath(Path path) {
        return new FunctionalCommand(
                () -> follower.follow(path),
                () -> {},
                interrupted -> {
                    if (interrupted) follower.stop();
                },
                () -> !follower.following(),
                drive);
    }

    private Command driveFor(double y, double x, double rx, long millis) {
        return drive.runEnd(() -> drive.drive(y, x, rx), drive::stop).withTimeout(millis);
    }

    private Command shoot(long millis) {
        return Commands.parallel(
                        shooter.setRPM(SHOOTER_RPM),
                        Commands.waitMillis(SPIN_UP_MILLIS)
                                .andThen(intake.run(FEED_POWER), indexer.feed(FEED_POWER)))
                .withTimeout(millis)
                .andThen(intake.run(0));
    }
}
