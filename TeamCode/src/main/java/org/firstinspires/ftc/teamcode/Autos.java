package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

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
    private Autos autos;

    private final Drive drive;
    private final Indexer indexer;
    private final Shooter shooter;
    private final Intake intake;
    private Follower follower;


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

//    Poses
    private final PoseFactory p = PoseFactory.degrees();

    public Command testingSequence() {
        return Commands.parallel(
                drive.bl(1).withTimeout(1),
                drive.bl(1).withTimeout(1),
                drive.bl(1).withTimeout(1),
                drive.bl(1).withTimeout(1),
                intake.run(1),
                Commands.waitMillis(2000),
                shooter.shoot(indexer).withTimeout(3000));
    }

    // Pedro poses are in inches and radians, x is forward and y is left of where the robot starts.
    public Command pathAuto() {
        Pose startPose = new Pose(0, 0, 0);
        Pose shootPose = new Pose(24, 0, 0);
        Pose park = new Pose(48, 24, 0);

        Path toShoot = line(startPose, shootPose).constant(0);
        Path toPark = curve(shootPose, new Pose(48, 0), park).tangent();

        return Commands.sequence(
                drive.runOnce(() -> follower.setPose(startPose)),
                followPath(toShoot),
                shooter.shoot(indexer),
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

    @Override
    public void initialize() {
        autos = new Autos(new Drive(hardwareMap), new Indexer(hardwareMap), new Shooter(hardwareMap), new Intake(hardwareMap));

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        followPath(Path );
    }

    @Override
    public void loop() {
        follower.update();
    }
}
