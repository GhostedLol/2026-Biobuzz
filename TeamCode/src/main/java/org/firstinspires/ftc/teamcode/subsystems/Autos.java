package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

/**
 * Autonomous routines. Each public method builds and returns one routine as a command, an OpMode
 * (see ExampleAuto) schedules it once the match starts.
 */
public class Autos {
    private static final double DRIVE_POWER = 0.4;
    private static final double SHOOTER_RPM = 1300;
    private static final double FEED_POWER = 0.5;

    /** Time for the shooter to get up to speed before the indexer starts feeding. */
    private static final long SPIN_UP_MILLIS = 1000;
    private static final long SHOOT_MILLIS = 4000;

    private final Drive drive;
    private final Indexer indexer;
    private final Shooter shooter;

    public Autos(Drive drive, Indexer indexer, Shooter shooter) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
    }

    /** Drives forward, shoots, then strafes right. All distances are timed, not measured. */
    public Command exampleAuto() {
        return Commands.sequence(
                driveFor(DRIVE_POWER, 0, 0, 1000),
                Commands.waitMillis(500),
                shoot(SHOOT_MILLIS),
                driveFor(0, DRIVE_POWER, 0, 1000));
    }

    /**
     * Drives at the given powers for a fixed time, then stops. Directions are field centric, with
     * forward being the way the robot faced when the OpMode was initialised.
     */
    private Command driveFor(double y, double x, double rx, long millis) {
        return drive.runEnd(() -> drive.drive(y, x, rx), drive::stop).withTimeout(millis);
    }

    /**
     * Spins the shooter up, then feeds with the indexer once {@link #SPIN_UP_MILLIS} has passed.
     * Neither of them finishes on its own, so the timeout is what ends this and stops both.
     */
    private Command shoot(long millis) {
        return Commands.parallel(
                        shooter.setRPM(SHOOTER_RPM),
                        Commands.waitMillis(SPIN_UP_MILLIS).andThen(indexer.feed(FEED_POWER)))
                .withTimeout(millis);
    }
}
