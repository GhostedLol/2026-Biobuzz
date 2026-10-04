package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class Main extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        boolean lastA = false;

        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

//        drive
        while (opModeIsActive()) {
            robot.drive.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

//            intake
            if (gamepad1.a && !lastA)
                robot.intake.toggle();
            lastA = gamepad1.a;

//            indexer
            if (gamepad1.b)
                robot.indexer.run(0.5);
            else
                robot.indexer.stop();

//            shooter
            if (gamepad1.right_trigger_pressed)
                robot.shooter.setRPM(200);
            else
                robot.shooter.stop();

            robot.update();

            telemetry.addData("Status", "Running");
            telemetry.addData("left",  "%.2f", robot.drive.getLeft());
            telemetry.addData("right", "%.2f", robot.drive.getRight());
            telemetry.update();
        }
    }
}
