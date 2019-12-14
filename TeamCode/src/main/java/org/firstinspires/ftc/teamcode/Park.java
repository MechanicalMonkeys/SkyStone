package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="park")
public class Park extends LinearOpMode {
    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, false);
        waitForStart();
        robot.drive.driveForwardDistance(3, 0.25, this);
    }
}
