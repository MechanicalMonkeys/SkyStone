package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Red", name = "Building Zone Red Close")
public class MM_BuildingZoneRedCLOSE extends LinearOpMode {
    private Robot robot = new Robot();
    //private Imu imu = new Imu();
    //private DistanceSensorMethods dsMethods = new DistanceSensorMethods();

    MM_BuildingZoneTemplateRed opmode = new MM_BuildingZoneTemplateRed(MM_BuildingZoneTemplateRed.ParkingPosition.CLOSE,
            MM_BuildingZoneTemplateRed.AllianceColor.RED, this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}