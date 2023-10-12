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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;


public class DriveSubsystem extends SubsystemBase{

    private final CANSparkMax m_frontLeftMotor;
    private final CANSparkMax m_frontRightMotor;
    private final CANSparkMax m_backLeftMotor;
    private final CANSparkMax m_backRightMotor;
    private final MotorController leftSideGroup;
    private final MotorController rightSideGroup;
    private final DifferentialDrive drive;

    public DifferentialDrive getDrive() {
        return drive;
    }

    public DriveSubsystem()
     {
        m_frontLeftMotor = new CANSparkMax(DrivebaseConstants.FRONT_LEFT_SPARK_ID, MotorType.kBrushed);
        m_frontRightMotor = new CANSparkMax(DrivebaseConstants.FRONT_RIGHT_SPARK_ID, MotorType.kBrushed);
        m_backLeftMotor = new CANSparkMax(DrivebaseConstants.BACK_LEFT_SPARK_ID, MotorType.kBrushed);
        m_backRightMotor = new CANSparkMax(DrivebaseConstants.BACK_RIGHT_SPARK_ID, MotorType.kBrushed);
      
        leftSideGroup = new MotorControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        rightSideGroup = new MotorControllerGroup(m_frontRightMotor, m_backRightMotor); 

        m_frontLeftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_frontRightMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_backLeftMotor.setIdleMode(DrivebaseConstants.BRAKE);
        m_backRightMotor.setIdleMode(DrivebaseConstants.BRAKE);

        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);
        m_backLeftMotor.setInverted(true);
        m_backRightMotor.setInverted(true);

         drive = new DifferentialDrive(leftSideGroup, rightSideGroup);
     }


    public MotorController getLeftSideGroup() {
        return leftSideGroup;
    }

    public MotorController getRightSideGroup() {
        return rightSideGroup;
    }

    public void arcadeDrive(double arcadeDriveSpeed, double arcadeDriveRotations) {
        getDrive().arcadeDrive(arcadeDriveSpeed, arcadeDriveRotations);
    }


    public void tankDrive(double leftSpeed, double rightSpeed) {
        getDrive().tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive(double xSpeed, double zRotations, boolean allowTurnInPlace) {
        getDrive().curvatureDrive(xSpeed, zRotations, allowTurnInPlace);
    }

    @Override
    public void periodic() {}
}
