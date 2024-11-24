package frc.robot.hijackablemovement;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Joystick implements MovementSource {
    @Override
    public double getR() {
        return -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(),
                Constants.OIConstants.kDriveDeadband);
    }

    public double getXSpeed() {
        return -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(),
                Constants.OIConstants.kDriveDeadband);
    }
}