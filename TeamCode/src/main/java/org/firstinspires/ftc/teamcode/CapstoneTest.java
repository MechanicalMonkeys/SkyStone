package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Capstone Test")
public class CapstoneTest extends LinearOpMode {
    private Servo capstoneArm = null;
    @Override
    public void runOpMode() {
        capstoneArm = hardwareMap.get(Servo.class, "capstoneArm");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_up && gamepad2.x) {
                armUp();
            } else if (gamepad2.dpad_down && gamepad2.x) {
                armDown();
            }
            telemetry.addData("Servo Position", capstoneArm.getPosition());
            telemetry.update();
        }
    }

    void armUp() {
        capstoneArm.setPosition(0.0);
    }

    void armDown() {
        capstoneArm.setPosition(0.95);
    }
}