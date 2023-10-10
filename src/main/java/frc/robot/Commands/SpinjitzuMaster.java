package frc.robot.Commands;
import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class SpinjitzuMaster extends CommandBase
{
    private final double speed;
    private  double kP;
    private  double kI;
    private  double P, I, D;
    private  double error;
    private double previousError;
    public SpinjitzuMaster(double speed, double kP, double kI)
    {
        this.kP = kP;
        this.kI = kI;
        this.speed = speed;
        addRequirements(Robot.getDrivebase());

        
    }

    @Override
    public void initialize()
    {
        Robot.LloydKaiJayZaneColeNyaNavx.reset();
    }

    @Override
    public boolean isFinished()
    {
        return Robot.XBOX_CONTROLLER.getLeftBumperPressed();
    }

    @Override
    public void execute()
    {
        error = -Robot.getNavx().getAngle();
        P = error * kP;
        D = (error-previousError) * kP;//WWWWWWW CODE
        I += error * kI;
        previousError = error;
        Robot.getDrivebase().arcadeDrive(speed, P+I+D);
        
    }

    //@Override
}
