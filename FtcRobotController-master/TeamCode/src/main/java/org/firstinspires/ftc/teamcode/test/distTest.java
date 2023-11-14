package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.Arrays;

@TeleOp
public class distTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetIMUHeading();


        waitForStart();

        while (opModeIsActive()) {
            driver.update();
            double dist = driver.getdistRight();
            double head = driver.pullIMUHeading();
            double distvalue=0;
            double[] distarray=new double[8];
            for (int i=0; i<8; i++) {
                //distvalue += driver.getdistRight();
                distarray[i] = driver.getdistRight();
            }
            double[] result = removeMinMax(distarray);
            for (int i=0; i<2; i++) {
                distvalue += result[i];
            }
            distvalue=distvalue/2;

            driver.drive((8.1-distvalue)/-3.5, 0, 0, false);

            /*if (distvalue > 8.5) {
                driver.drive(0.3, 0, -head/40, false);
            } else if (dist < 8 ){
                driver.drive(-0.3, 0, -head/40, false);
            } else {
                driver.drive(0, 0, -head/40, false);
            }

             */

            telemetry.addData("dist", dist);
            telemetry.addData("tuned dist", distvalue);
            telemetry.addData("imu", head);
            telemetry.update();

        }
    }

    public static double[] removeMinMax(double[] arr) {
        Arrays.sort(arr);
        return Arrays.copyOfRange(arr, 3, arr.length - 3);
    }
}
