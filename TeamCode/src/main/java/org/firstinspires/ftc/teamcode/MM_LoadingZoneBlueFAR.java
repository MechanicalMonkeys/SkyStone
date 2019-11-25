package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group =  "Blue", name = "Loading Zone Blue Far")
public class MM_LoadingZoneBlueFAR extends LinearOpMode {

    private Robot robot = new Robot();
    MM_LoadingZoneTemplate opmode = new MM_LoadingZoneTemplate(MM_LoadingZoneTemplate.ParkingPosition.FAR,
            MM_LoadingZoneTemplate.AllianceColor.BLUE, this, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        opmode.runOpMode();
    }
}