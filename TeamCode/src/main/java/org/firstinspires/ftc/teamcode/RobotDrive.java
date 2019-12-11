package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

public class RobotDrive {
    Robot robot;
    DcMotor rearLeft;
    DcMotor rearRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    public RobotDrive(Robot robot, DcMotor rearLeft, DcMotor rearRight, DcMotor frontLeft, DcMotor frontRight) {
        this.robot = robot;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
    }

    void setDrivePower(double power) {
        /* sets all drive motors to a certain power */
        this.rearLeft.setPower(power);
        this.frontLeft.setPower(power);
        this.rearRight.setPower(power);
        this.frontRight.setPower(power);
    }

    void setDriveMode(DcMotor.RunMode runMode) {
        /* sets all drive motors to a certain mode */
        this.rearLeft.setMode(runMode);
        this.frontLeft.setMode(runMode);
        this.rearRight.setMode(runMode);
        this.frontRight.setMode(runMode);
    }

    void stopDrive() {
        /* stops all the drive motors */
        this.setDrivePower(0);
    }


    void driveForwardDistance(double distance, double power, LinearOpMode opmode) { // make power negative to go backwards
        /* drives forward a certain distance(in) using encoders */
        double targetAngle = robot.getHeading();

        // calculate ticks
        long NUM_TICKS_LONG = StrictMath.round(this.robot.TICKS_PER_INCH * distance);
        int NUM_TICKS = (int) NUM_TICKS_LONG;

        // reset encoders
        this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode
        this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive
        while (opmode.opModeIsActive() && Math.abs(this.rearLeft.getCurrentPosition()) < NUM_TICKS && Math.abs(this.frontLeft.getCurrentPosition()) < NUM_TICKS
                && Math.abs(this.rearRight.getCurrentPosition()) < NUM_TICKS && Math.abs(this.frontRight.getCurrentPosition()) < NUM_TICKS) {
            double currentAngle = this.robot.getHeading();
            double error = Math.tanh((currentAngle - targetAngle) / 100);
            this.frontLeft.setPower(power + error);
            this.frontRight.setPower(power - error);
            this.rearLeft.setPower(power + error);
            this.rearRight.setPower(power - error);
            opmode.telemetry.addData("Angle of Robot", currentAngle);
            opmode.telemetry.update();
        }

        // stop driving
        this.stopDrive();

    }

    void driveWithDistanceSensor(String mode, double distanceForSensor, double power, DistanceSensor distanceSensor, LinearOpMode opmode) {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        double sign = Math.signum(distance - distanceForSensor);
        double targetAngle = robot.getHeading();
        switch(mode) {
            case "strafe":
                while (distance * sign > distanceForSensor * sign) {
                    double currentAngle = this.robot.getHeading();
                    double error = Math.tanh((currentAngle - targetAngle) / 30); // we have to constrain the error between -1 and 1
                    this.rearLeft.setPower(-power * sign + error);
                    this.frontLeft.setPower(power * sign + error);
                    this.rearRight.setPower(power * sign - error);
                    this.frontRight.setPower(-power * sign - error);
                    distance = distanceSensor.getDistance(DistanceUnit.INCH);
                }
                break;
            case "drive":
                while (distance * sign > distanceForSensor * sign) {
                    double currentAngle = this.robot.getHeading();
                    double error = Math.tanh((currentAngle - targetAngle) / 100);
                    this.frontLeft.setPower(power * sign + error);
                    this.frontRight.setPower(power * sign - error);
                    this.rearLeft.setPower(power * sign + error);
                    this.rearRight.setPower(power * sign - error);
                    opmode.telemetry.addData("Angle of Robot", currentAngle);
                    opmode.telemetry.update();
                    distance = distanceSensor.getDistance(DistanceUnit.INCH);
                }
                break;
        }
        this.stopDrive();
    }

    void setStrafe(double power) {
        /* strafes at certain power
        positive power goes to the right
        negative power goes to the left */
        this.rearLeft.setPower(-power);
        this.frontRight.setPower(-power);

        this.frontLeft.setPower(power);
        this.rearRight.setPower(power);
    }

    void strafeTime(double power, long milliseconds, LinearOpMode opmode) throws InterruptedException {
        /* strafes for a certain amount of milliseconds */
        double targetAngle = this.robot.getHeading(); // you want to stay at this angle the whole time
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opmode.opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < milliseconds) {
            double currentAngle = this.robot.getHeading();
            double error = Math.tanh((currentAngle - targetAngle) / 30); // we have to constrain the error between -1 and 1
            this.rearLeft.setPower(-power + error);
            this.frontLeft.setPower(power + error);
            this.rearRight.setPower(power - error);
            this.frontRight.setPower(-power - error);
        }
        this.stopDrive();
    }

    void turnRight(double power, long milliseconds) throws InterruptedException {
        this.rearLeft.setPower(power);
        this.frontLeft.setPower(power);

        this.rearRight.setPower(-power);
        this.frontRight.setPower(-power);
        Thread.sleep(milliseconds);
        this.stopDrive();
    }

    void turnWithImu(double power, double angle, LinearOpMode opmode) {
        double currentAngle = this.robot.getHeading();
        double targetAngle = currentAngle + angle;
        double sign = Math.signum(angle);
        this.rearLeft.setPower(-power * sign);
        this.frontLeft.setPower(-power * sign);
        this.rearRight.setPower(power * sign);
        this.frontRight.setPower(power * sign);
        while (opmode.opModeIsActive() && currentAngle * sign <= targetAngle * sign) {
            currentAngle = this.robot.getHeading();
            opmode.telemetry.addData("Angle of Robot", currentAngle);
            opmode.telemetry.update();
        }
        this.stopDrive();
    }

    void turnToGlobalPosition(double power, double angle, LinearOpMode opmode) {
        double angleToTurn = angle - this.robot.getHeading();
        this.turnWithImu(power, angleToTurn, opmode);
    }

    void driveUntilColor(String mode, double power, String color, LinearOpMode opmode) {
        double targetAngle = robot.getHeading();
        switch(mode) {
            case "strafe":
                switch(color) {
                    case "red":
                        while (opmode.opModeIsActive() && this.robot.insideColor.red() < 4000) {
                            double currentAngle = this.robot.getHeading();
                            double error = Math.tanh((currentAngle - targetAngle) / 30); // we have to constrain the error between -1 and 1
                            this.rearLeft.setPower(-power + error);
                            this.frontLeft.setPower(power + error);
                            this.rearRight.setPower(power - error);
                            this.frontRight.setPower(-power - error);
                        }
                        break;
                    case "blue":
                        while (opmode.opModeIsActive() && this.robot.insideColor.blue() < 4000) {
                            double currentAngle = this.robot.getHeading();
                            double error = Math.tanh((currentAngle - targetAngle) / 30); // we have to constrain the error between -1 and 1
                            this.rearLeft.setPower(-power + error);
                            this.frontLeft.setPower(power + error);
                            this.rearRight.setPower(power - error);
                            this.frontRight.setPower(-power - error);
                        }
                        break;
                }
                break;
            case "drive":
                /* drives forward a certain distance(in) using encoders */

                // calculate ticks
                long NUM_TICKS_LONG = StrictMath.round(this.robot.TICKS_PER_INCH * 999999);
                int NUM_TICKS = (int) NUM_TICKS_LONG;

                // reset encoders
                this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // set mode
                this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                switch(color) {
                    case "red":
                        // drive
                        while (opmode.opModeIsActive() && this.robot.insideColor.red() < 4000) {
                            double currentAngle = this.robot.getHeading();
                            double error = Math.tanh((currentAngle - targetAngle) / 40);
                            this.frontLeft.setPower(power + error);
                            this.frontRight.setPower(power - error);
                            this.rearLeft.setPower(power + error);
                            this.rearRight.setPower(power - error);
                            opmode.telemetry.addData("Angle of Robot", currentAngle);
                            opmode.telemetry.update();
                        }
                        // stop driving
                        break;
                    case "blue":
                        // drive
                        while (opmode.opModeIsActive() && this.robot.insideColor.blue() < 4000) {
                            double currentAngle = this.robot.getHeading();
                            double error = Math.tanh((currentAngle - targetAngle) / 40);
                            this.frontLeft.setPower(power + error);
                            this.frontRight.setPower(power - error);
                            this.rearLeft.setPower(power + error);
                            this.rearRight.setPower(power - error);
                            opmode.telemetry.addData("Angle of Robot", currentAngle);
                            opmode.telemetry.update();
                        }
                        break;
                }
                break;
            default:
                return;
        }
        this.stopDrive();
    }
}
