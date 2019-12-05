package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


// THIS IS NOT AN OPMODE - IT IS A DEFINING CLASS

public class Imu implements ImuConstants {
    // Motors
    private DcMotor bl; // Back left
    private DcMotor br; // Back right
    private DcMotor fl; // Front left
    private DcMotor fr; // Front right
    private OpMode opmode;

    BNO055IMU imu;

    @Override
    public void init(OpMode opmode, boolean initmotors) {
        this.opmode = opmode;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // IMU Init
        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (initmotors) this.initMotors(opmode);
    }

    private void initMotors(OpMode opmode) {
        // Motor mapping
        this.bl = opmode.hardwareMap.dcMotor.get("rearLeft");
        this.fl = opmode.hardwareMap.dcMotor.get("frontLeft");
        this.br = opmode.hardwareMap.dcMotor.get("rearRight");
        this.fr = opmode.hardwareMap.dcMotor.get("frontRight");

        // Motor direction
        this.bl.setDirection(DcMotor.Direction.FORWARD);
        this.fl.setDirection(DcMotor.Direction.FORWARD);
        this.br.setDirection(DcMotor.Direction.REVERSE);
        this.fr.setDirection(DcMotor.Direction.REVERSE);
    }

    // Method for getting heading
    @Override
    public double getHeading() {
        // State used for updating telemetry
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //opmode.telemetry.addData("Heading", angles.firstAngle);
        //opmode.telemetry.update();
        return angles.firstAngle; // a smaller angle (negative) means tilting to the right
    }

    // Methods for accurate drive
    @Override
    public void driveForwardCorrection(double power, double initialAngle, double delta) {
        // This method makes the robot adjust its heading (by turning) while going forward.
        // This allows the robot to go in a perfectly straight line.
        // This is a very simple, yet practical, algorithm.

        //double delta = 3.0;

        // Start the motors
        this.bl.setPower(power);
        this.fl.setPower(power);
        this.br.setPower(power);
        this.fr.setPower(power);

        if (power > 0.0) { // positive power
            if ((this.getHeading() - delta ) < initialAngle) { // too far right, turn right motors
                this.bl.setPower(0.0); // stop
                this.fl.setPower(0.0); // stop
                this.br.setPower(power * 2); // run
                this.fr.setPower(power * 2); // run
            } else if ((this.getHeading() + delta) > initialAngle) { // too far left, turn left motors
                this.br.setPower(0.0); // stop
                this.fr.setPower(0.0); // stop
                this.bl.setPower(power * 2); // run
                this.fl.setPower(power * 2); // run
            } else { // perfectly straight, go forward
                this.br.setPower(power); // run
                this.fr.setPower(power); // run
                this.bl.setPower(power); // run
                this.fl.setPower(power); // run
            }
        } else { // negative power, reverse controls
            if ((this.getHeading() - delta ) < initialAngle) { // too far right, turn left motors
                this.br.setPower(0.0); // stop
                this.fr.setPower(0.0); // stop
                this.bl.setPower(power * 2); // run
                this.fl.setPower(power * 2); // run
            } else if ((this.getHeading() + delta ) > initialAngle) { // too far left, turn right motors
                this.bl.setPower(0.0); // stop
                this.fl.setPower(0.0); // stop
                this.br.setPower(power * 2); // run
                this.fr.setPower(power * 2); // run
            } else { // perfectly straight, go forward
                this.br.setPower(power); // run
                this.fr.setPower(power); // run
                this.bl.setPower(power); // run
                this.fl.setPower(power); // run
            }
        }
    }

    @Deprecated
    void strafeCorrection(double power, double initialAngle) { // Buggy code (for now)
        // Start the motors
        this.bl.setPower(-power);
        this.fr.setPower(-power);
        this.fl.setPower(power);
        this.br.setPower(power);

        if (this.getHeading() < initialAngle) { // too far right, turn left motors
            double P_CONSTANT = 0.1;
            double p = Math.abs(this.getHeading());

            this.bl.setPower(-power); // run
            this.fr.setPower(-(power + (p * P_CONSTANT))); // faster
            this.fl.setPower(power + (p * P_CONSTANT)); // run
            this.br.setPower(power); // faster
        } else if (this.getHeading() > initialAngle) { // too far left, turn right motors
            double P_CONSTANT = 0.1;
            double p = Math.abs(this.getHeading());

            this.bl.setPower(-(power + (p * P_CONSTANT))); // faster
            this.fr.setPower(-power); // run
            this.fl.setPower(power); // faster
            this.br.setPower(power + (p * P_CONSTANT)); // run
        } else { // perfectly straight, keep strafing
            this.bl.setPower(-power); // run
            this.fr.setPower(-power); // run
            this.fl.setPower(power);  // run
            this.br.setPower(power);  // run
        }
    }

    @Override
    public void turnToPosition(double power, double angle, double direction) {
        if (direction == 1) { // turn right motors forward and left motors backward
            while (this.getHeading() < angle) {
                this.bl.setPower(-power);
                this.fl.setPower(-power);
                this.br.setPower(power);
                this.fr.setPower(power);
            }
            // Stop motors
            this.bl.setPower(0.0);
            this.fl.setPower(0.0);
            this.br.setPower(0.0);
            this.fr.setPower(0.0);
        } else if (direction == -1) { // turn left motors forward and right motors backward
            while (this.getHeading() > -angle) {
                this.bl.setPower(power);
                this.fl.setPower(power);
                this.br.setPower(-power);
                this.fr.setPower(-power);
            }
            // Stop motors
            this.bl.setPower(0.0);
            this.fl.setPower(0.0);
            this.br.setPower(0.0);
            this.fr.setPower(0.0);
        }
    }

    @Override
    public void turnDegrees(double power, double angle, int direction) {
        double initialHeading = this.getHeading();
        if (direction == 1) { // turn right motors forward and left motors backward
            while (this.getHeading() < initialHeading + angle) {
                this.bl.setPower(-power);
                this.fl.setPower(-power);
                this.br.setPower(power);
                this.fr.setPower(power);
            }
            // Stop motors
            this.bl.setPower(0.0);
            this.fl.setPower(0.0);
            this.br.setPower(0.0);
            this.fr.setPower(0.0);
        } else if (direction == -1) { // turn left motors forward and right motors backward
            while (this.getHeading() > (-initialHeading) - (-angle)) {
                this.bl.setPower(power);
                this.fl.setPower(power);
                this.br.setPower(-power);
                this.fr.setPower(-power);
            }
            // Stop motors
            this.bl.setPower(0.0);
            this.fl.setPower(0.0);
            this.br.setPower(0.0);
            this.fr.setPower(0.0);
        }
    }

    @Override
    public void stopMotors() {
        this.bl.setPower(0.0);
        this.fl.setPower(0.0);
        this.br.setPower(0.0);
        this.fr.setPower(0.0);
    }
}
