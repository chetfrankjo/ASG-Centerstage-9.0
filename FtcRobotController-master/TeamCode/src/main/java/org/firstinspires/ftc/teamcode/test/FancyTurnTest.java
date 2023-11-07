package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class FancyTurnTest extends LinearOpMode {
    Pose2d currentPos;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, true);

        waitForStart();

        while (opModeIsActive()) {
            driver.update();
            currentPos = driver.getCurrentPos();
            double deltaX = 75-currentPos.getX();
            double deltaY = 0-currentPos.getY();
            //double relativeXToPoint = deltaX * Math.cos(worldAngle_rad);
            //double relativeYToPoint = deltaY * Math.sin(worldAngle_rad);

            double reltheta = Math.atan2(deltaX,deltaY);
            reltheta = MathFunctions.AngleWrap(reltheta-Math.toRadians(currentPos.getHeading()));
            System.out.println(Math.atan2(2,0));
            double dist = Math.hypot(deltaX, deltaY);
            double relativeXToPoint = dist*Math.sin(reltheta);
            double relativeYToPoint = dist*Math.cos(reltheta);
            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            //System.out.println(movementXPower + " : " + movementYPower);
            //movement_x = ((movementXPower*-deltaY)+(movementYPower*deltaX)) * 0.25;
            //movement_y = ((movementYPower*deltaY)+(movementXPower*deltaX)) * 0.25;
            driver.drive(movementXPower*0.4, movementYPower*0.4, 0, false);

        }
    }
}
