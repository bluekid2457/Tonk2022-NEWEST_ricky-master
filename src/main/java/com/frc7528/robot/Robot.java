//https://docs.wpilib.org/en/stable/index.html for documentation

/*
Running the program:
press ctrl + shift + p
look up "WPILib: Simulate Robot Code on Desktop"
press "halsim_gui.dll"
assign desired system joystick to virtual joystick
click "Map Gamepad"
click "Teleoperated"
*/

package com.frc7528.robot;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import java.lang.Math;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.io.File;
import java.text.SimpleDateFormat;
//import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import static com.frc7528.robot.common.RobotMap.*;


//imports from wpilib and RobotMap class

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {
		
	private SendableChooser<Double> fineControlSpeed = new SendableChooser<>();
	private SendableChooser<Double> deadBandOptions = new SendableChooser<>();
	private double fineControlSpeedDouble;
	private SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yy HH:mm:ss");
	//private NetworkTableEntry ledStatusEntry = Shuffleboard.getTab("DRIVETRAIN").add("LED Status", "OFF").getEntry();
	private final Drivetrain m_drivetrain = new Drivetrain();
	
	private double conveyor1;
	private double conveyor2;
	private double Tmove;
	private double RAft;
	private double LAft;
	private boolean found;
	private double target_angle=0;
	private double k;
	private static double H_tape = 2.5;
	private static double H_lime = 0.76;
	private static double ang_lime = 30;
	private static boolean fadeAway = false;
	private static boolean shoot = false;


	//UsbCamera cam0 = CameraServer.startAutomaticCapture(0);
	//MjpegServer switchCam = CameraServer.addSwitchedCamera("Camera");

	//UsbCamera cam = new UsbCamera(, path)
	//CvSink cvSink = cam.getVideo();
	
	//private final RamseteController m_ramsete = new RamseteController();
	private static Timer m_timer = new Timer();
	//private Trajectory m_trajectory;
	//private final XboxController m_controller = new XboxController(0);
	//private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
	//private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
	//creates necessary objects to run program

	private boolean isInverted = true;
	
	//Configure Victors, SendableChoosers, and initial debug statistics
	@Override
	public void robotInit() {
		File file = new File(Robot.class.getProtectionDomain().getCodeSource().getLocation().getPath());
		Shuffleboard.getTab("DEBUG").add("Left Aft Drivetrain Firm",m_leftAft.getFirmwareVersion());
		Shuffleboard.getTab("DEBUG").add("Left Front Drivetrain Firm",m_leftFront.getFirmwareVersion());
		Shuffleboard.getTab("DEBUG").add("Right Aft Drivetrain Firm",m_rightAft.getFirmwareVersion());
		Shuffleboard.getTab("DEBUG").add("Right Front Drivetrain Firm",m_rightFront.getFirmwareVersion());
		Shuffleboard.getTab("DEBUG").add("Last code deploy",sdf.format(file.lastModified()));
		Shuffleboard.getTab("DEBUG").add("Pigeon IMU", pidgey);
		Shuffleboard.getTab("DEBUG").add("Angle", testPrint1);
		Shuffleboard.getTab("DEBUG").add("Compass Heading", testPrint2);
		Shuffleboard.getTab("DEBUG").add("Yaw", testPrint3);

		System.out.println(testPrint1);
		System.out.println(testPrint2);
		System.out.println(testPrint3);
			
		//Format all motor controllers
		m_leftAft.configFactoryDefault();
		m_leftFront.configFactoryDefault();
		m_rightAft.configFactoryDefault();
		m_rightFront.configFactoryDefault();
		ConveyorMotor1.configFactoryDefault();
		ConveyorMotor2.configFactoryDefault();
		Turret.configFactoryDefault();

		FlyL_Aft.configFactoryDefault(); //change this
		FlyR_Front.configFactoryDefault();
		//Config followers
		m_leftAft.follow(m_leftFront);
		m_rightAft.follow(m_rightFront);
		
		//Config inversion
		m_leftFront.setInverted(false);
		m_rightFront.setInverted(true);
		m_rightAft.setInverted(true);

		//C14.configFactoryDefault();
		//C9.configFactoryDefault();
		
		//Instantiate DifferentialDrive and put it on Shuffleboard
		m_drive = new DifferentialDrive(m_leftFront,m_rightFront);
		Shuffleboard.getTab("DRIVETRAIN").add(m_drive);
		
		//Put Limelight LED Status to Shuffleboard
		//ledStatusEntry.setString("OFF");
		
		//Fine Control Speed chooser
		fineControlSpeed.addOption("35% Speed", 0.35);
		fineControlSpeed.addOption("40% Speed", 0.40);
		fineControlSpeed.setDefaultOption("45% Speed", 0.45);
		fineControlSpeed.addOption("50% Speed", 0.50);
		fineControlSpeed.addOption("55% Speed", 0.55);
		fineControlSpeed.addOption("60% Speed", 0.60);
		Shuffleboard.getTab("SETUP").add("Fine Control Speed", fineControlSpeed);
		
		//Deadband chooser
		deadBandOptions.setDefaultOption("5%", 0.05);
		deadBandOptions.addOption("10%", 0.10);
		deadBandOptions.addOption("15%", 0.15);
		Shuffleboard.getTab("SETUP").add("Dead Band", deadBandOptions);
		
		//Transmits video through cameras
		//CameraServer.startAutomaticCapture();

		// Flush NetworkTables every loop. This ensures that robot pose and other values
		// are sent during every iteration.
		setNetworkTablesFlushEnabled(true);
		//List<Translation2d> waypoints = new ArrayList<Translation2d>();
		//Generates trajectory for the display using the starting pose, given waypoints, and a trajectory config
		//m_trajectory = TrajectoryGenerator.generateTrajectory(
		//new Pose2d(2, 2, new Rotation2d()),
		
		//List.of(new Translation2d(3,3), new Translation2d(4,5), new Translation2d(5,4), new Translation2d(6,6)),
		//new Pose2d(4, 3, new Rotation2d()),
		//new TrajectoryConfig(2, 2));
			
	}

	@Override
	public void robotPeriodic() {
		m_drivetrain.periodic();
	}

	/*
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
	}
	*/

	@Override
	public void autonomousPeriodic() {
	
	}

	//Sets fine control speed and deadband
	@Override
	public void teleopInit() {
		fineControlSpeedDouble = -fineControlSpeed.getSelected(); //Set fine control speed
		m_drive.setDeadband(deadBandOptions.getSelected()); //Set deadband
		limelightTable.getEntry("ledMode").setNumber(0);
		System.out.println("yo");
		target_angle = pidgey.getAngle();
	}

	//make method get angle from limelight y value
	public static void shootBall(){ // conveyor 1 and 2 negative fly both legative
		if (m_timer.get() < 2){
			FlyL_Aft.set(ControlMode.PercentOutput, -0.6);
			FlyR_Front.set(ControlMode.PercentOutput, -0.6);
			if (m_timer.get() > 1){
				double conveyer2 = -0.5;
				double conveyor1 = -0.5;
				ConveyorMotor1.set(ControlMode.PercentOutput, conveyor1);
				ConveyorMotor2.set(ControlMode.PercentOutput, conveyer2);
			}
			
			shootBall();
		}
		
	}
	public static double getDistance(){
		try{
			
			double Theta_t = limelightTable.getEntry("ty").getDouble(0);
			
			
			//System.out.println(" - ty");
			Math.tan(Math.toRadians(Theta_t+ang_lime));
			
			double dist = (H_tape - H_lime)/Math.tan(Math.toRadians(Theta_t+ang_lime));
			
			dist = dist *39.37;
			dist *= 1.11;
			dist -= 2;
			System.out.println(dist + " - distance    -----    " + Theta_t + " - ang lime");

			return dist;
		} catch (Exception e) {
			System.out.println("dist error"+ e);
			return 0;
		}

	}
	//Teleop driving (Fine control and joystick control)
	public static void shootFadeAway(double Target_distance){
		double dist = getDistance();
		//System.out.println(dist);

		double error = Target_distance - dist;
		System.out.println(error + " - error");
		if (error > 15){
			m_drive.curvatureDrive(-0.15, 0 , false);
		}
		else if (error < -10){
			m_drive.curvatureDrive(0.15, 0 , false);
		}
		else if (error > 10.0){
			m_drive.curvatureDrive(-0.15, 0 , false);
		}
		//else if (error < -5.0){
		//	m_drive.curvatureDrive(0.20, 0 , false);
		//}
		else {
			fadeAway = false;
			m_timer.reset();
			//shootBall();
		}
		

	}
	@Override
	public void teleopPeriodic() {
		PhotonPipelineResult result = cam1.getLatestResult();
		//PhotonTrackedTarget ball = 
		if (m_joy.getRawButton(7)){
			System.out.println(cam1.getLEDMode());
			
			System.out.println(result.hasTargets() + " --- " + result.getLatencyMillis());
			double yaw = result.getBestTarget().getYaw();
			System.out.println( yaw + " -- yaw");
			if (yaw>5){
				m_drive.curvatureDrive(0, -0.1, true);
			}
			else if (yaw < 5){
				m_drive.curvatureDrive(0,0.1,true);
			}
		}

		//m_drive.curvatureDrive(0, 0, false);
		if (m_joy.getRawButton(3)){
			m_drive.curvatureDrive(0, m_joy.getZ()*-0.5, true);
		}
		

		//m_drive.curvatureDrive((m_joy.getThrottle()+1)*0.25, 0, false);
		//System.out.println((m_joy.getThrottle()+1)*0.25);
		Tmove = 0;
		if (m_joy.getRawButtonPressed(6)){
			System.out.println("hello");
			fadeAway = true;
		} 
		if (m_joy.getRawButtonPressed(4)){
			fadeAway = false;
		}
		if (fadeAway){
			shootFadeAway(100);
		}
		if (m_joy.getRawButton(3)){
			getDistance();

		}
		//Force LimeLight Off
		if (m_joy.getRawButton(12)) { // 6
			limelightTable.getEntry("ledMode").setNumber(1);
			found = false;
		}

		//Force Limelight On and Start LimeLight Tracking
		if (m_joy.getRawButton(11)) {
			System.out.print(limelightTable.getEntry("tx").getDouble(0));
			System.out.println(" - tx");
			System.out.print(limelightTable.getEntry("ta").getDouble(0));
			System.out.println("- ta");
			//System.out.print(limelightTable.getEntry("tv").getDouble(0));
			//System.out.println("- tv");
			limelightTable.getEntry("ledMode").setNumber(3);
			
			//positive is right
			if (limelightTable.getEntry("tx").getDouble(0)>0.0) {
				Tmove = -0.2;
			} else if (limelightTable.getEntry("tx").getDouble(0)<0.0) {
				Tmove = 0.2;
			}

			if (limelightTable.getEntry("tv").getDouble(0)==1.0) {
				//m_drive.arcadeDrive(0.1, 0);
				found = true;
				System.out.println("hey");
				System.out.println();
			} else if(limelightTable.getEntry("tv").getDouble(0)==0) {
				found = false;
			}

			if (found) {
				FlyR_Front.set(ControlMode.PercentOutput, 0.5);
			}
		}

		//Conveyor Belt System
		conveyor1 = 0.0;
		if (m_joy.getRawButton(11)){
			conveyor1 = -0.5;
		}
		conveyor2 = 0.0;
		if (m_joy.getRawButton(11)){
			conveyor2 = -0.5;
		}
		
		//Turret Control
		if (m_joy.getRawButton(11)){// button 6
			Tmove = -0.2;
		}
		if (m_joy.getRawButton(11)){ // button 5
			Tmove = 0.2;
		}
			
		//Force Flywheel Motors
		RAft = 0;
		LAft = 0; //Laft
		if (m_joy.getRawButton(11)){ // button 4
			RAft = -0.1;
			RAft = m_joy.getThrottle()*-1;
		}
		if (m_joy.getRawButton(11)){ // button 3
			LAft = -0.1; //LAFT
			LAft = m_joy.getThrottle()*-1; //Laft

		}
		
		Turret.set(ControlMode.PercentOutput, Tmove);

		ConveyorMotor2.set(ControlMode.PercentOutput, conveyor2);
		ConveyorMotor1.set(ControlMode.PercentOutput, conveyor1);

		FlyL_Aft.set(ControlMode.PercentOutput, LAft);////////FlyL_Aft not C9
		FlyR_Front.set(ControlMode.PercentOutput, RAft);
			
		//Fine control
		if (m_joy.getPOV() == 0) { //Forward
				m_drive.arcadeDrive(fineControlSpeedDouble,0);
		} else if (m_joy.getPOV() == 270) { //Right
				m_drive.arcadeDrive(0,-fineControlSpeedDouble);
		} else if (m_joy.getPOV() == 180) { //Backward
				m_drive.arcadeDrive(-fineControlSpeedDouble,0);
		} else if (m_joy.getPOV() == 90) { //Left
				m_drive.arcadeDrive(0,fineControlSpeedDouble);
		} else {

			//Pidgey control
			/*
			if (m_joy.getRawButton(10)) { 
				heading = pidgey.getAngle();
				System.out.println(heading + " - head");
				double error = 0;

				if (heading > 0) {
					error = 0.3;
				} else if (heading < 0) {
					error = -0.3;
				}
				System.out.println(error + " - eror");

				//double error  = heading;
				if (isInverted) {
						m_drive.curvatureDrive(m_joy.getY(), -m_joy.getX() +error  , m_joy.getRawButton(2));
				} else {
						m_drive.curvatureDrive(-m_joy.getY(), -m_joy.getX() +error, m_joy.getRawButton(2));
				}
				*/

				if (m_joy.getRawButton(5)) { 	
					double turn = 0;
					k = 0.03;
					double sensitivity = 5;
					heading = pidgey.getAngle();
					target_angle += m_joy.getX()*sensitivity*m_joy.getY()*-1;
					System.out.println(target_angle);
					//target_angle = 0;
					double error = 0;
					error = heading-target_angle;
					turn = error*k;
					/*k = 0.005;
					thresh = 3;
					double error = heading;
					if (Math.abs(error)>thresh) {
						double turn = error*k;
					}
					*/
					//double error  = heading;
					if (isInverted) {
							m_drive.curvatureDrive(m_joy.getY(), turn , m_joy.getRawButton(2));
					} else {
							m_drive.curvatureDrive(-m_joy.getY(), turn , m_joy.getRawButton(2));
					}
					
			}
		}
	}

}