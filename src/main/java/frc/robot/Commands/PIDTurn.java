package frc.robot.Commands;

import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDTurn extends CommandBase{
    
    private PIDController pid;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0.1;

    public PIDTurn(double setpoint) {
       
        pid = new PIDController(kP, kI, kD);
        pid.setSetpoint(setpoint);
        pid.enableContinuousInput(-180,180);
        pid.setTolerance(0.5);

        addRequirements(Robot.getDrivebase());

    }

    @Override
    public void execute() {
        Robot.getDrivebase().arcadeDrive(0, pid.calculate((Robot.getNavX().getAngle())));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}
