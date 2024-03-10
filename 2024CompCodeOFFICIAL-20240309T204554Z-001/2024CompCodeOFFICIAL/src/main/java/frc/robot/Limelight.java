package frc.robot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import org.opencv.core.Mat;


import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;




public class Limelight { 

    private TeleopSwerve teleDrive;
    private Robot robot;
    public Constants constants;
    private Swerve swerveDrive;
    private SwerveModule Smod;

    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry tv;


    private final double minX;
    private final double maxX;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    

    public Limelight(Robot robot, double min, double max) {
        this.robot = robot;

    

    


       // Mat image = cameraStream.getImage(null);


    /* public while(true){}
        Mat Frame = camera.getImage();

         // Detect AprilTags using official WPILib library
        AprilTagDetector detector = new AprilTagJNIDetector();
        List<AprilTagDetection> detections = detector.detect(frame);
*/
    

        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        targetArea = table.getEntry("ta");
        tv = table.getEntry("tv");
        double tx = table.getEntry("aprilTag.x").getDouble(0.0);
        double ty = table.getEntry("aprilTag.y").getDouble(0.0);
        double ta = table.getEntry("aprilTag.area").getDouble(0.0); // Check for tag presence

        minX = min;
        maxX = max;
    }

     public void positionRobot(double left, double right){

        double tx = table.getEntry("aprilTag.x").getDouble(0.0);
        double ty = table.getEntry("aprilTag.y").getDouble(0.0);
        double ta = table.getEntry("aprilTag.area").getDouble(0.0); // Check for tag presence

        double x = getX();

        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = rotationSup.getAsDouble();

        if (tx > 0){ //x >= getMaxX()
            // I saved Sabrina's code but I did not save this
            //robot.driveTrain.driveFX(robot.maxSpeed, robot.maxSpeed);
            swerveDrive.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true); // right

        }
        else if (tx < 0){
            swerveDrive.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            -rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true); // left
        } 
        else{
            swerveDrive.resetModulesToAbsolute();
        } 
       
    }
        
       /*  if (ta > 0.0) { // Only drive if a tag is detected
            double turn = tx * 0.05; // Adjust coefficient for desired turn speed
            double forward = 0.5; // Adjust forward speed as needed

            // Calculate individual module speeds and angles for swerve drive
            double[] speeds = setSpeed(turn, forward); // Implement your calculation method
            double[] angles = calculateModuleAngles(tx, ty); // Implement your calculation method

            Smod.setSpeed(true, true);
            Smod.setAngle(true);
        } else {
            swerveDrive.resetModulesToAbsolute(); // Stop if no tag detected
            Smod.resetToAbsolute();
        }
    } */

    public class AprilTagLimelightTurn {

        private CameraServer camera;
        private AprilTagDetector aprilTagDetector;
        private NetworkTable limelight;
        private NetworkTableEntry txEntry;
        private NetworkTableEntry tyEntry;
        private DifferentialDrive drive;
    
        private double turnTolerance = 0.5; // Degrees within which to stop turning
        private double turnSpeed = 0.4; // Turn speed, adjust as needed
    
        public AprilTagLimelightTurn() {
            // Initialize camera, AprilTag detector, Limelight connection, and drivetrain
            // ...
        }
    /* 
        public void run() {
            while (true) {
                // Capture camera frame
                Mat frame = CameraServer();
    
                // Detect AprilTags
                AprilTagDetection[] detections = aprilTagDetector.detect(frame);
    
                if (detections.length == 0) {
                    // Get first AprilTag for example
                    AprilTagDetection tag = detections[0];
    
                    // Get Limelight data
                    double tx = txEntry.getDouble(0.0);
    
                    // Calculate horizontal angle error
                    double horizontalAngleError = tx;
    
                    // Turn towards the AprilTag
                    if (Math.abs(horizontalAngleError) > turnTolerance) {
                        drive.arcadeDrive(0, turnSpeed * Math.signum(-horizontalAngleError));
                    } else {
                        drive.stopMotor();
                    }
    
                    // Display information on SmartDashboard
                    SmartDashboard.putNumber("Horizontal Angle Error", horizontalAngleError);
                    SmartDashboard.putBoolean("Target Aligned", Math.abs(horizontalAngleError) <= turnTolerance);
                } else {
                    // Handle no detection
                    drive.stopMotor(); // Or potentially search for the tag
                }

            }
        } */
    }

    public void setLight(boolean state) {   
        /*
         * The part that says "state ? 3 : 1"
         * is basically saying that true = 3 and false = 1.
         * If state is true the number is 3.
         * If state is false the number is 1.
         * 
         * - AJ
         */
        table.getEntry("ledMode").setNumber(state ? 3 : 1);
        table.getEntry("camMode").setNumber(0);
    }

    public Mat CameraServer() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'CameraServer'");
    }

    public double getX() {
        tx = table.getEntry("tx");
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return tx.getDouble(0.0);

    }

    public void setPipeline(int pipeline) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(pipeline);
      }

    public double getY() {
        ty = table.getEntry("ty");
        SmartDashboard.putNumber("y", ty.getDouble(0.0));
        return ty.getDouble(0.0);        
    }

    public double getTargetArea(){
        targetArea = table.getEntry("ta");
        return targetArea.getDouble(0.0);
    }
    
    public double getV(){
        tv = table.getEntry("tv");
        return tv.getDouble(0);
    }

    public double getMinX(){
        return minX;
    }
    public double getMaxX(){
        return maxX;
    }

    public static boolean getTV(String string) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTV'");
    }

    public static double getTX(String string) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTX'");
    }
}


/*targets info    
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx"); //horizontal offset
NetworkTableEntry ty = table.getEntry("ty"); //Vertical offset
NetworkTableEntry ta = table.getEntry("ta"); // Target area

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);


//post to smart dashboard periodically
public void updateLimelightValues() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    table.getEntry("ledMode").setNumber(1);
}

    
} */
