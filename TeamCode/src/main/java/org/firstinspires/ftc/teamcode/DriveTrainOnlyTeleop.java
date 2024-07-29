package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Drivetrain Only", group="Robot")
//@Disabled
public class DriveTrainOnlyTeleop extends LinearOpMode{

    RobotHardware robot = new RobotHardware();

    public static final double DRIVE_SPEED = 0.5;

    public void runOpMode(){
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            while(gamepad1.dpad_up){
                robot.setMotorPowers(-DRIVE_SPEED);
            }
            while(gamepad1.dpad_down){
                robot.setMotorPowers(DRIVE_SPEED);
            }
            while(gamepad1.dpad_left){
                robot.setMotorPowers(-DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, -DRIVE_SPEED);
            }
            while(gamepad1.dpad_right){
                robot.setMotorPowers(DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED, DRIVE_SPEED);
            }
            robot.setMotorPowers(0.0);
            robot.setMotorPowers(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x,
                    -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x,
                    -gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x,
                    gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x);
        }
        robot.setMotorPowers(0.0);
    }


}
