package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AutoAlign;


public class Telemetry extends SubsystemBase{
    private static Telemetry telemetry = null;
    private Field2d field2d;
    public ChassisSpeeds currentVelocity;
    private Drive drivetrain = Drive.getInstance();
    private AutoAlign align = AutoAlign.getInstance();
    
    public Telemetry(){
        field2d = new Field2d();
        SmartDashboard.putData("Field ", field2d);
    }

    public void update(){
        //Send match time to the dashboard
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        //Update robot position on the 2d field inside the dashboard
        field2d.setRobotPose(drivetrain.getPose());

        //Update robot driving velocity
        currentVelocity = drivetrain.getRobotVelocity();
        double botVelocity = Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
        SmartDashboard.putNumber("Velocity", botVelocity);
        // SmartDashboard.putNumber("Encoder1", drivetrain.getEncoderOffset(0));
        // SmartDashboard.putNumber("Encoder2", drivetrain.getEncoderOffset(1));
        // SmartDashboard.putNumber("Encoder3", drivetrain.getEncoderOffset(2));
        // SmartDashboard.putNumber("Encoder4", drivetrain.getEncoderOffset(3));
        SmartDashboard.putNumber("Motor Vel:", drivetrain.getMotorVel(0));
        SmartDashboard.putNumber("Motor Volts:", drivetrain.getMotorVel(0));
        SmartDashboard.putNumber("Hub Distance:", align.getHubDist().baseUnitMagnitude());
    }

    public static Telemetry getInstance(){
        if (telemetry == null){
            telemetry = new Telemetry();
        }
        return telemetry;
    }
}