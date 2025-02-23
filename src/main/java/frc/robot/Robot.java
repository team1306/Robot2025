// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.FieldLocation;
import frc.robot.dashboardv2.Dashboard;
import frc.robot.util.Dashboard.DashboardHelpers;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
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
    }

    @Override
    public void disabledInit() {
        robotContainer.alianceLEDs();
        new WaitCommand(2).andThen(new InstantCommand(robotContainer::zeroTargetPositions)).ignoringDisable(true).schedule();
    }

    @Override
    public void disabledPeriodic() {
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
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
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
