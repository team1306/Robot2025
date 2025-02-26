// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.FieldLocation;
import frc.robot.dashboardv2.Dashboard;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import frc.robot.util.Elastic;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private final Timer gcTimer = new Timer();

    @Override
    public void robotInit() {
        DashboardHelpers.addUpdateClass(this);
        FieldLocation.calculateReefPositions();
        robotContainer = new RobotContainer();
        robotContainer.alianceLEDs();
        robotContainer.chainLeds();
        System.gc();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        DashboardHelpers.update();
        Dashboard.update();
        if (gcTimer.advanceIfElapsed(5)) {
            System.gc();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.alianceLEDs();
        new WaitCommand(2).andThen(new InstantCommand(robotContainer::resetTargetPositions)).ignoringDisable(true).schedule();
    }

    @GetValue
    public static boolean run;
    public static Runnable runnable;
    
    private final Elastic.Notification notification = new Elastic.Notification(
            Elastic.Notification.NotificationLevel.INFO, "Reset Odometry", "Odometry has been reset to the current path's initial pose");

    @Override
    public void disabledPeriodic() {
        if(run && runnable != null){
            runnable.run();
            run = false;
            SmartDashboard.putBoolean("Robot/run", false);
            Elastic.sendNotification(notification);
        }
    }

    @Override
    public void disabledExit() {
        FieldLocation.calculateReefPositions();
    }

    @Override
    public void autonomousInit() {
        robotContainer.alianceLEDs();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        // robotContainer.drivebase.pushPID = true;
        robotContainer.alianceLEDs();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
