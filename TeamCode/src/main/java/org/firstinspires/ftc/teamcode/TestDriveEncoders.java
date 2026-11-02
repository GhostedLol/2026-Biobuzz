package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Checks the drive motor encoders and measures ticks per inch for pedro/Constants.java.
 * Press A to zero, push the robot by hand exactly MEASURE_INCHES forward and read "forward",
 * then zero again, push it the same distance LEFT and read "strafe".
 * A wheel stuck at 0 while pushing means its encoder cable is not plugged in.
 */
@TeleOp(name = "Drive Encoder Test", group = "Diagnostics")
public class TestDriveEncoders extends LinearOpMode {
    private static final double MEASURE_INCHES = 48;

    @Override
    public void runOpMode() {
        DcMotor fl = motor(Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontLeftDirection.get());
        DcMotor fr = motor(Constants.drivetrainConfig.frontRightName.get(), Constants.drivetrainConfig.frontRightDirection.get());
        DcMotor bl = motor(Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backLeftDirection.get());
        DcMotor br = motor(Constants.drivetrainConfig.backRightName.get(), Constants.drivetrainConfig.backRightDirection.get());
        int[] zero = new int[4];

        while (!isStopRequested()) {
            int[] now = {fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};
            if (gamepad1.a) System.arraycopy(now, 0, zero, 0, 4);
            int dFl = now[0] - zero[0], dFr = now[1] - zero[1], dBl = now[2] - zero[2], dBr = now[3] - zero[3];

            double forward = (dFl + dFr + dBl + dBr) / 4.0;
            double strafe = -(dFl - dFr - dBl + dBr) / 4.0;

            telemetry.addLine("A = zero. Push exactly " + (int) MEASURE_INCHES + " in forward, then left.");
            telemetry.addData("frontLeft", dFl);
            telemetry.addData("frontRight", dFr);
            telemetry.addData("backLeft", dBl);
            telemetry.addData("backRight", dBr);
            telemetry.addData("forward ticks/inch", "%.2f", forward / MEASURE_INCHES);
            telemetry.addData("strafe ticks/inch", "%.2f", strafe / MEASURE_INCHES);
            telemetry.update();
        }
    }

    private DcMotor motor(String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        return motor;
    }
}
