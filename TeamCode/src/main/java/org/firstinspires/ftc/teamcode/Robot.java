package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    private final Drive drive;
    private final Intake intake;
    private final Indexer indexer;
    private final Shooter shooter;

    private final GamepadEx driver;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();

        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        driver = new GamepadEx(gamepad);

        CommandScheduler.getInstance().schedule(new RunCommand(this::robotPeriodic));

//      drive
        drive.setDefaultCommand(
                drive.run(() -> drive.drive(
                        -gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x)));

        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(drive::resetYaw);

//      intake
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intake::toggle);

//      indexer
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(indexer.run(0.5)).whenReleased(indexer.stop());

    //      shooter
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(shooter.setRPM(1000)).whenReleased(shooter.stop());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void robotPeriodic() {
        intake.update();

        TelemetryPacket packet = new TelemetryPacket();
        drive.addTelemetry(packet);
        intake.addTelemetry(packet);
        indexer.addTelemetry(packet);
        shooter.addTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
