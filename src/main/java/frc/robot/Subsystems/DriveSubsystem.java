package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class DriveSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;
    private final MotorController leftSideGroup;
    private final MotorController rightSideGroup;
    private final DifferentialDrive drive;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    //private final DifferentialDriveKinematics kinematics;

    public DifferentialDrive getDrive() {
        return drive;
    }

    public DriveSubsystem()
     {
        m_leftMotor = new CANSparkMax(DrivebaseConstants.LEFT_SPARK_ID, MotorType.kBrushed);
        m_rightMotor = new CANSparkMax(DrivebaseConstants.RIGHT_SPARK_ID, MotorType.kBrushed);
      
        leftSideGroup = new MotorControllerGroup(m_leftMotor);
        rightSideGroup = new MotorControllerGroup(m_rightMotor); //:(
        
        leftEncoder = m_leftMotor.getEncoder();
        rightEncoder = m_rightMotor.getEncoder();
        /*
        
        kinematics = new DifferentialDriveKinematics(
				new Translation2d(-0.49276 / 2.0, -0.23 / 2.0),
				new Translation2d(-0.49276 / 2.0, 0.23 / 2.0),
				new Translation2d(0.49276 / 2.0, -0.23 / 2.0),
				new Translation2d(0.49276 / 2.0, 0.23 / 2.0));
        
        */
        m_leftMotor.setInverted(DrivebaseConstants.LEFT_SPARK_INVERTED);
        m_rightMotor.setInverted(DrivebaseConstants.RIGHT_SPARK_INVERTED);

        m_leftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_rightMotor.setIdleMode(DrivebaseConstants.BRAKE);

        drive = new DifferentialDrive(leftSideGroup, rightSideGroup);
     }

    public CANSparkMax getM_leftMotor() {
        return m_leftMotor;
    }

    public CANSparkMax getM_rightMotor() {
        return m_rightMotor;
    }

    public MotorController getLeftSideGroup() {
        return leftSideGroup;
    }

    public MotorController getRightSideGroup() {
        return rightSideGroup;
    }

    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    public RelativeEncoder getleftEncoder()
    {
        return leftEncoder;
    }
    public RelativeEncoder getrightEncoder()
    {
        return rightEncoder;
    }

    public double getLeftEncoderPosition() {
        return Math.abs(leftEncoder.getPosition());
    }

    public double getRightEncoderPosition() {
        return Math.abs(rightEncoder.getPosition());
    }

    public double getAverageEncoders()
    {
        return Math.abs((Math.abs(getLeftEncoderPosition()) + Math.abs(getRightEncoderPosition())) / 2);
    }

    public void resetEncoders()
    {
        leftEncoder.setPosition(0.0D);
        rightEncoder.setPosition(0.0D);

    }

    public void resetEncoders(double value)
    {
        leftEncoder.setPosition(value);
        rightEncoder.setPosition(value);
    }


    public void arcadeDrive(double arcadeDriveSpeed, double arcadeDriveRotations)
    {
        getDrive().arcadeDrive(arcadeDriveSpeed, arcadeDriveRotations);
    }


    public void tankDrive(double leftSpeed, double rightSpeed) 
    {
        getDrive().tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive(double xSpeed, double zRotations, boolean allowTurnInPlace)
    {
        getDrive().curvatureDrive(xSpeed, zRotations, allowTurnInPlace);
    }


    public Pose2d getPose() {
        return pose;
    }

    //public DifferentialDriveKinematics getKinematics() {
    //  return kinematics;
    //}

    @Override
    public void periodic()
    {
        // LEFT_ENCODER_ENTRY.setDouble(getLeftEncoder());
		// RIGHT_ENCODER_ENTRY.setDouble(getRightEncoder());
		// ENCODER_DISTANCE_ENTRY.setDouble(getAverageEncoders());
		// NAVX_ANGLE_ENTRY.setDouble(Robot.navX.getAngle());
		// NAVX_RATE_ENTRY.setDouble(Robot.navX.getRate());
    }
}
