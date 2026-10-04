package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled

public class MikeRowBat extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor intakeMotor;
    private DcMotor indexerMotor;
    private DcMotor shooterMotor;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double toilet = 0.5;
        boolean poo = false;
        boolean pee = false;
        double rpm = 400;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        indexerMotor = hardwareMap.get(DcMotor.class, "indexerMotor");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            left  = drive + turn;
            right = drive - turn;

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            frontLeft.setPower(left);
            frontRight.setPower(right);
            backLeft.setPower(left);
            backRight.setPower(right);
//            intake
            if (gamepad1.a && !pee)
                poo = !poo;
            pee = gamepad1.a;

            if (poo)
                intakeMotor.setPower(toilet);
            else
                intakeMotor.setPower(0);
//            indexer
            if (gamepad1.b)
                indexerMotor.setPower(0.5);
            else
                indexerMotor.setPower(0);
//            shooter
            if (gamepad1.right_trigger_pressed)
                shooterMotor.setVelocity(rpm * 6.0, AngleUnit.DEGREES);
            else
                shooterMotor.setPower(0);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("left", left);
            packet.put("right", right);
            packet.put("intakePower", intakeMotor.getPower());
            packet.put("indexerPower", indexerMotor.getPower());
            packet.put("shooterPower", shooterMotor.getPower());

            packet.put("flPower", frontLeft.getPower());
            packet.put("frPower", frontRight.getPower());
            packet.put("blPower", backLeft.getPower());
            packet.put("brPower", backRight.getPower());

            packet.put("shooterRPM", shooterMotor.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Status", "Running");
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();
        }
    }
}
