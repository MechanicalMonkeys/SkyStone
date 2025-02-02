
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Halo Drive")
public class DriveHalo extends OpMode {

    Robot robot = new Robot();

    // init variables
    double speedControl = 0.5; // to make the robot go slower since we use TorqueNado 20s
    private double compensation = 1; // compensation so the robot can move forward AND turn while both joysticks are used
    private float deadZone = 0.2f; // joystick deadzone
    private boolean armClosed = false;
    boolean slowMode = false; // activate slowMode if both joysticks are pushed down
    boolean strafeMode = false;
    boolean turboMode = false;
    Boolean[] buttons = new Boolean[7];
    double wristPosition = 0.85;

    @Override
    public void init() {
        robot.init(this, false);
        gamepad1.setJoystickDeadzone(deadZone);
        for (int i = 0; i < buttons.length; i++) {
            buttons[i] = false;
        }
        robot.arm.rotateGripper(wristPosition);
        robot.capstoneArm.setPosition(1.0);

        telemetry.addData("Initialized", "Ready to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        this.driveController();
        this.armController();
        this.wristController();
        this.gripperController();
        this.liftController();
        this.waffleController();
        this.capstoneArmController();
        telemetry.addData("Robot Info: ", robot.getInfo());
        telemetry.update();
    }

    void liftController() {
        double ry = gamepad1.right_stick_y;
        if (ry > 0.5) { // lift logic
            robot.liftDown();
        } else if (ry < -0.5) {
            robot.liftUp();
        } else {
            robot.stopLift();
        }
    }

    void waffleController() {
        if (gamepad1.x && !buttons[6]) {
            robot.moveWaffleMover(true);
        }
        if (gamepad1.y && !buttons[0]) {
            robot.moveWaffleMover(false);
        }
        buttons[0] = gamepad1.y;
        buttons[6] = gamepad1.x;
    }

    void armController() {
        /*if (gamepad2.dpad_down && !buttons[1]) {
            try {
                robot.bringArmDown(this);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (gamepad2.dpad_up && !buttons[2]) {
            try {
                robot.foldArmBack(this);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }*/ //if (robot.armPos == Robot.armPosition.ACTIVE) {
            robot.arm.setArmRotatePower(0.4 * gamepad2.left_stick_y);
        //}
    }

    void wristController() {
        if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) {
            this.wristPosition -= 0.01 * gamepad2.right_stick_y;
            robot.arm.rotateGripper(this.wristPosition);
        } else if (gamepad2.b && !buttons[3]) {
            robot.arm.toggleWrist();
            wristPosition = robot.arm.gripperRotateServo.getPosition();
        }
        buttons[3] = gamepad2.b;

    }

    void gripperController() {
        if (gamepad2.a && !buttons[4]) {
            if (armClosed) {
                robot.arm.releaseBlock(); // release the block
            } else {
                robot.arm.gripBlock(); // grab the block
            }
            armClosed = !armClosed;
        }
        buttons[4] = gamepad2.a;
    }

    void driveController() {
        this.slowMode = gamepad1.right_bumper;
        this.turboMode = gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5;
        this.strafeMode = gamepad1.left_bumper;

        if (this.slowMode) {
            speedControl = 0.25;
        } else if (this.turboMode) {
            speedControl = 1;
        } else {
            speedControl = 0.5;
        }

        if (this.strafeMode) {
            if (this.slowMode) {
                speedControl = 0.35;
            }
            robot.drive.setStrafe(speedControl * gamepad1.right_stick_x);
        } else {
            double drive = speedControl * -gamepad1.left_stick_y; // forward
            double turn = 0.5 * speedControl * gamepad1.right_stick_x; // turn
            double leftPower = Range.clip(drive + turn + compensation * turn, -1, 1);
            double rightPower = Range.clip(drive - turn - compensation * turn, -1, 1);
            robot.rearLeft.setPower(leftPower); // assign powers to motors
            robot.frontLeft.setPower(leftPower);
            robot.rearRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
        }
    }

    void capstoneArmController() {
        if ((gamepad2.dpad_up && gamepad2.x) && !buttons[5]) {
            robot.toggleCapstoneArm();
        }
        buttons[5] = gamepad2.dpad_up && gamepad2.x;
    }
}
