// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import badgerlog.Dashboard;
import badgerlog.DashboardConfig;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.FieldLocation;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private final Timer gcTimer = new Timer();

    @Override
    public void robotInit() {
        Dashboard.initialize(DashboardConfig.defaultConfig);
        FieldLocation.recalculateFieldPositions();
        robotContainer = new RobotContainer();
        System.gc();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Dashboard.update();
        if (gcTimer.advanceIfElapsed(5)) {
            System.gc();
        }
    }

    @Override
    public void disabledInit() {
        // robotContainer.setAllianceLed();
        robotContainer.setSeizureMode();
        // robotContainer.setRainbow();
        new WaitCommand(2).andThen(new InstantCommand(robotContainer::resetTargetPositions)).ignoringDisable(true).schedule();
    }
    
    @Override
    public void disabledPeriodic() {
    
    }

    @Override
    public void disabledExit() {
        FieldLocation.recalculateFieldPositions();
    }

    @Override
    public void autonomousInit() {

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
