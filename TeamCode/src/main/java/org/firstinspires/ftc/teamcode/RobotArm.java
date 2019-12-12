package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class RobotArm {
    Robot robot;
    DcMotor armRotate;
    Servo gripperRotateServo;
    Servo grabServo;

    public RobotArm(Robot robot, DcMotor armRotate, Servo gripperRotateServo, Servo grabServo) {
        this.robot = robot;
        this.armRotate = armRotate;
        this.gripperRotateServo = gripperRotateServo;
        this.grabServo = grabServo;
    }

    void moveArmRotate(int targetPosition, double power, LinearOpMode opmode) {
        double sign = Math.signum(targetPosition);

        // reset encoders
        this.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power
        this.setArmRotatePower(power);

        // wait for the armRotate motors to reach the position or else things go bad bad
        ElapsedTime timer = new ElapsedTime();
        while (this.armRotate.getCurrentPosition() * sign <  targetPosition * sign && opmode.opModeIsActive()) {
            // if it takes more than 2 seconds, something is wrong so we exit the loop
            if (timer.time(TimeUnit.SECONDS) > 10) {
                opmode.telemetry.addData("Error", "Gripper movement took too long");
                opmode.telemetry.update();
                break;
            }

            opmode.telemetry.addData("Gripper", targetPosition + " " + this.armRotate.getCurrentPosition());
            opmode.telemetry.update();
        }

        // stop the armRotate motors
        this.stopArmRotate();
    }

    void rotateGripper(double position) {
        this.gripperRotateServo.setPosition(position);
        //this.gripperRotateServo2.setPosition(position);
    }

    void bringArmDown(LinearOpMode opmode) {
        if (robot.armPos == Robot.armPosition.REST) { // we only bring the arm down if the arm is resting
            // we rotate the arm 180 + ANGLE_OF_GRIPPER_WHEN_GRABBING degrees
            this.moveArmRotate(-3800, 1.0, opmode);
            this.robot.armPos = Robot.armPosition.ACTIVE;
        }
    }

    void gripBlock() {
        this.grabServo.setPosition(0.6);
        this.robot.gripperPos = Robot.gripperPosition.CLOSED;
    }

    void releaseBlock() {
        this.grabServo.setPosition(1);
        this.robot.gripperPos = Robot.gripperPosition.OPEN;
    }

    void pickUpBlock(LinearOpMode opmode) throws InterruptedException { // for autonomous
        this.bringArmDown(opmode); // bring arm down
        Thread.sleep(500);
        // we rotate the gripper so it is parallel to the ground
        this.rotateGripper(0.9);
        this.gripBlock(); // grab the block
        Thread.sleep(500);
        // we rotate the gripper back
        this.rotateGripper(1.0);
    }

    void setArmRotatePower(double power) {
        this.armRotate.setPower(power);
    }

    void stopArmRotate() { this.setArmRotatePower(0); }

    void toggleWrist() {
        this.robot.gripperRotatePosition = 1.85 - this.robot.gripperRotatePosition;
        this.rotateGripper(this.robot.gripperRotatePosition);
    }

    void grabBlockAuto() throws InterruptedException {
        // grab block
        this.rotateGripper(0.85);
        Thread.sleep(250);
        this.gripBlock();
        Thread.sleep(250);
        this.rotateGripper(1.0);
    }
}
