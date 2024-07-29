/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Turn using gyro and PID tuning
 */

@Autonomous(name="Robot: Auto Drive By Gyro", group="Robot")
//@Disabled
public class RobotAutoDriveByGyro_Linear extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    static final double HEADING_ERROR_THRESHOLD = 1.0;
    static final double INITIAL_MOTOR_POWER = 0.3;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        robot.setDrivetrainMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

//        turn(90.0);
//        sleep(3000);
//        turnTo(150.0);
//        sleep(3000);
//        turnPID(-120.0);
//        sleep(3000);
        turnToPID(90.0);
    }

    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//TODO: May need to change axes order depending on control hub
        currAngle = 0.0;
    }

    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if(deltaAngle > 180.0) deltaAngle -= 360.0;
        else if(deltaAngle <= -180.0) deltaAngle += 360.0;

        currAngle += deltaAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", "Current Angle: %.7f", orientation.firstAngle);
        telemetry.update();

        return currAngle;
    }

    public void turn(double degrees){

        double target = getAngle() + degrees;
        double error = degrees;
        while(opModeIsActive() && Math.abs(error) > HEADING_ERROR_THRESHOLD){
            double motorPower = (error < 0.0)? -INITIAL_MOTOR_POWER : INITIAL_MOTOR_POWER;
            robot.setMotorPowers(-motorPower, -motorPower, motorPower, motorPower);
            error = getAngle() - target;

            telemetry.addData("error", "%.7f degrees", error);
            telemetry.update();
        }
        robot.setMotorPowers(0.0);
    }

    public void turnTo(double targetAngle){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = orientation.firstAngle - targetAngle;
        if(error > 180.0) error -= 360.0;
        else if(error <= -180.0) error += 360.0;

        turn(error);
    }

    public void turnToPID(double targetAngle){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = orientation.firstAngle - targetAngle;
        if(error > 180.0) error -= 360.0;
        else if(error <= -180.0) error += 360.0;

        PIDControllerTurn pidCT = new PIDControllerTurn(targetAngle, PIDConstants.kP,
                PIDConstants.kI, PIDConstants.kD); // TODO: Change values to proper numbers
        while(opModeIsActive() && Math.abs(error) > HEADING_ERROR_THRESHOLD){
            double motorPower = pidCT.update(getAngle()) * Math.signum(error);
            robot.setMotorPowers(-motorPower, -motorPower, motorPower, motorPower);

            telemetry.addData("Target IMU Angle", targetAngle);
            telemetry.addData("Current IMU Angle", getAngle());
            telemetry.addData("Error", error);
            telemetry.update();

            error = getAngle() - targetAngle;
        }
        robot.setMotorPowers(0.0);
    }
    public void turnPID(double degrees){
        turnToPID(degrees + getAngle());
    }

}
