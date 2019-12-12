package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Distance Sensor Test", group = "Sensor")
public class DistanceSensorTest extends LinearOpMode {
    private Robot robot = new Robot();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Forward Distance", String.format("%.01f cm", robot.frontDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Left Distance", String.format("%.01f cm", robot.leftDistance.getDistance(DistanceUnit.CM)));
            //telemetry.addData("Right Distance", String.format("%.01f cm", robot.rightDistance.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
    }
}