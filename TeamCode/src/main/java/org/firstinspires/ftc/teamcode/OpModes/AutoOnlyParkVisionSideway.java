package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VisionEasyOpenCV;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * RoboRaisers # 21386 Autonomous using EasyOpenCV and Park only mode
 */
@Autonomous(name = "RoboRaiders ParkOnly Sideways", group = "00-Autonomous", preselectTeleOp = "RoboRaiders TeleOp")
@Disabled
public class AutoOnlyParkVisionSideway extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    private VisionEasyOpenCV visionEasyOpenCV;
    private OpenCvCamera camera;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";
    public DriveTrain driveTrain;

    private DcMotor Arm;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        visionEasyOpenCV = new VisionEasyOpenCV();

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on INIT.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.setPipeline(visionEasyOpenCV);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            telemetry.clearAll();
            telemetry.addData("Start RoboRaiders Autonomous Mode adopted for Team","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Park Position Identified by Camera: ", visionEasyOpenCV.getPosition());
            telemetry.update();
        }
        //Set DEFAULT as LEFT
        VisionEasyOpenCV.ParkingPosition position = VisionEasyOpenCV.ParkingPosition.ONE;

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            //Set the arm to low junction height
            Arm.setTargetPosition(1000);
            Arm.setPower(0.3);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Stop VisionEasyOpenCV process
            position = visionEasyOpenCV.getPosition();
            camera.stopStreaming();

            //Build parking trajectory based on last detected target by visionEasyOpenCV
            buildParking(position);
            driveTrain.getLocalizer().setPoseEstimate(initPose);

            //run Autonomous trajectory
            runParking(position);
        }

        //Trajectory is completed, display Parking complete
        parkingComplete(position);
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d parkPose;

    //Build parking trajectory based on target detected by visionEasyOpenCV
    public void buildParking(VisionEasyOpenCV.ParkingPosition position){
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-68, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-38, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(-38, 60, Math.toRadians(0)); break; // Location 1
                    case TWO: parkPose = new Pose2d(-34, 34, Math.toRadians(0)); break; // Location 2
                    case THREE: parkPose = new Pose2d(-38, 11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-68, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-38, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(-38, -11, Math.toRadians(0)); break; // Location 1
                    case TWO: parkPose = new Pose2d(-34, -34, Math.toRadians(0)); break; // Location 2
                    case THREE: parkPose = new Pose2d(-38, -60, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(68, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(38, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(38, -60, Math.toRadians(180)); break; // Location 1
                    case TWO: parkPose = new Pose2d(34, -34, Math.toRadians(180)); break; // Location 2
                    case THREE: parkPose = new Pose2d(38, -11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                initPose = new Pose2d(68, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(38, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(position){
                    case ONE: parkPose = new Pose2d(38, 11, Math.toRadians(180)); break; // Location 1
                    case TWO: parkPose = new Pose2d(34, 34, Math.toRadians(180)); break; // Location 2
                    case THREE: parkPose = new Pose2d(38, 60, Math.toRadians(180)); break; // Location 3
                }
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(initPose)
                .strafeRight(15)
                .forward(5)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runParking(VisionEasyOpenCV.ParkingPosition position){
        telemetry.setAutoClear(false);
        telemetry.addData("Running RoboRaiders Autonomous Mode adopted for Team:","21386");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryParking);
    }

    public void parkingComplete(VisionEasyOpenCV.ParkingPosition position){
        telemetry.addData("Parked in Location", position);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) AutoOnlyPark adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}

