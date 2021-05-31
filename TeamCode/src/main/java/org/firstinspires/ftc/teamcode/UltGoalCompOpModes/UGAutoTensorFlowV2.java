/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.UltGoalCompOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="COMP Auto Tensor Flow", group="Linear Opmode")
//@Disabled
public class UGAutoTensorFlowV2 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private double ringCount;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo foundGrabber = null;

    private static final String VUFORIA_KEY =
            "AQE1Pez/////AAABmUdBbpjZMEyVqbtRwTqVAawIuV6+swu0GG2rkaVAvD9mstRHYZK3Xl9Th2EeecvqkApSlWkC4ALqL5Aq/oJFjJUVjPBnXbOJm/ouVG65BDsRVczB0PeiRLCYmCrJ72mYZMXKfP8BrlF0KEnu+rRvgvr09gLA20JjrYXPiqwbbkcBYq4NrY597cFXk7gwP+yzSnx6jo7gzQ+e4jbqySC8R5Y4NndZ//2C/ll5gLJ6bu2Su9ZkIIDlhHImGTJjsFuTWSVOd6ABR3m/EZsUBkwUqUut9slnNQXd6N3xS/VdkooANaJJebAroz16T1ebiCYAA0WKXXumnlxZ3NBs/6xkyYdT9eIQHNreQus3iVdHeQBD";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void wait(double secs) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (t.seconds() < secs && opModeIsActive()){}
    }

    public void straight(double power, double secs) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (t.seconds() < secs && opModeIsActive()){
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void turn(double powerLeft, double powerRight, double secs) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (t.seconds() < secs && opModeIsActive()){
            leftFront.setPower(powerLeft);
            leftBack.setPower(powerLeft);
            rightFront.setPower(powerRight);
            rightBack.setPower(powerRight);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void strafe(double power, double secs) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(t.seconds() < secs && opModeIsActive()) {
            leftFront.setPower(power);
            leftBack.setPower(power*-1);
            rightFront.setPower(power*-1);
            rightBack.setPower(power);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public double getAngle() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void turnTo(double degrees, double powerLeft, double powerRight) {
        if(getAngle()<degrees) {
            while(getAngle()<degrees) {
                leftFront.setPower(powerLeft);
                leftBack.setPower(powerLeft);
                rightFront.setPower(powerRight);
                rightBack.setPower(powerRight);
                telemetry.addData("Angle Value", getAngle());
                telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
        else {
            while(getAngle()>degrees) {
                leftFront.setPower(powerLeft);
                leftBack.setPower(powerLeft);
                rightFront.setPower(powerRight);
                rightBack.setPower(powerRight);
                telemetry.addData("Angle Value", getAngle());
                telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    public void turnBy(double degrees, double powerLeft, double powerRight) {
        if(powerLeft<powerRight) {
            double targetX = getAngle()+degrees;
            while(getAngle()<targetX) {
                leftFront.setPower(powerLeft);
                leftBack.setPower(powerLeft);
                rightFront.setPower(powerRight);
                rightBack.setPower(powerRight);
                telemetry.addData("Angle Value", getAngle());
                telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
        else {
            double targetX = getAngle()-degrees;
            while(getAngle()>targetX) {
                leftFront.setPower(powerLeft);
                leftBack.setPower(powerLeft);
                rightFront.setPower(powerRight);
                rightBack.setPower(powerRight);
                telemetry.addData("X Value", getAngle());
                telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    private int getRings() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                /**telemetry.addData("# Object Detected", updatedRecognitions.size());**/
                if (updatedRecognitions.size() == 0 ) {
                    // empty list.  no objects recognized.
                    /**telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone", "A");**/
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                       /** telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());**/

                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals("Single")) {
                            return 1;
                        } else if (recognition.getLabel().equals("Quad")) {
                            return 4;
                        } else {
                            return 0;
                        }
                    }
                }
            }
        }
        return 0;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.75, 16.0/9.0);

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "fl_motor");
        leftBack  = hardwareMap.get(DcMotor.class, "bl_motor");
        rightFront = hardwareMap.get(DcMotor.class, "fr_motor");
        rightBack = hardwareMap.get(DcMotor.class, "br_motor");
        foundGrabber = hardwareMap.get(Servo.class, "found_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Boolean autoEnd = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !autoEnd) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFPower = leftFront.getPower();
            double rightFPower = rightFront.getPower();
            double leftBPower = leftBack.getPower();
            double rightBPower = rightBack.getPower();

            strafe(-0.3,1.2);

            telemetry.addData("Rings:", ringCount);

            for( int i = 0; i<10; i++) {
                wait(0.1);
                ringCount += getRings();
                telemetry.addData("Rings:", ringCount);
                telemetry.update();
            }
            ringCount /= 10;
            ringCount += 0.5;
            ringCount = (int)ringCount;

            telemetry.addData("Rings:", ringCount);
            telemetry.update();

            tfod.shutdown();

            if(ringCount == 0) {
                //Close Score Zone
                strafe(0.3,1.2);
                turn(0.425,0.3,2.0);
              
            }
            else if (ringCount == 1) {
                //Mid Score Zone
                strafe(-0.3, 2);
            }
            else {
                //Far Score Zone
                strafe(0.3,2);
            }

            autoEnd = true;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left Front (%.2f), left Back (%.2f), right Front (%.2f), right Back (%.2f)", leftFPower, leftBPower, rightFPower, rightBPower);
            telemetry.update();
        }
    }
}
//snack bot