package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Blue", name = "Building Zone Blue Far")
public class MM_BuildingZoneBlueFAR extends LinearOpMode {
    private Robot robot = new Robot();
    //private Imu imu = new Imu();
    //private DistanceSensorMethods dsMethods = new DistanceSensorMethods();

    MM_BuildingZoneTemplateBlue opmode = new MM_BuildingZoneTemplateBlue(MM_BuildingZoneTemplateBlue.ParkingPosition.FAR,
            MM_BuildingZoneTemplateBlue.AllianceColor.BLUE, this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}