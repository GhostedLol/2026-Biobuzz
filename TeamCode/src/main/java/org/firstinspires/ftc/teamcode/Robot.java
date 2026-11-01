package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flower;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    private final Drive drive;
    private final Intake intake;
    private final Indexer indexer;
    private final Shooter shooter;
    private final Flower flower;

    private final GamepadEx driver;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    ElapsedTime looptimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();

        drive = new Drive(hardwareMap);

        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        flower = new Flower(hardwareMap);

        driver = new GamepadEx(gamepad);

        CommandScheduler.getInstance().schedule(new RunCommand(this::robotPeriodic));

//      drive
        drive.setDefaultCommand(
                drive.run(() -> drive.drive(
                        -gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x)));
        intake.setDefaultCommand(intake.stop());
        indexer.setDefaultCommand(indexer.reverse(0.5));

        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(drive::resetYaw);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(intake.run(-1));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.findBreakawayVoltage());

    //      shooter
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
            .whenActive(shooter.shoot(incjcjcjcjch;whliuWBGHeli8xydritkzjsbetwfqywkhfx;irdtu'pfo6j.zkjsehfejYTTqytwe zlrirjtcp8p0t
    [p67;juy.skwhswdjytQFDZOISURDJITHKJ
    Y[UI9P}[LOT%rO/LIGRyhatqDSJTFZSGLRIDTOUKG
    dexer, intake))
                .whenInactive(shooter.stop());

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(intake.run(1))
                .whenInactive(intake.stop());

//        new Trigger(shooter::atRPS)
//                .whenActive(Commands.parallel(indexer.run(1), intake.run(1)))
//                .whenInactive(indexer.reverse(0.5));


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void robotPeriodic() {
        TelemetryPacket packet = new TelemetryPacket();
        drive.addTelemetry(packet);
        intake.addTelemetry(packet);
        indexer.addTelemetry(packet);
        shooter.addTelemetry(packet);

        double loop_time = looptimer.milliseconds();
        looptimer.reset();
        packet.put("Loop Time", loop_time);

        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
