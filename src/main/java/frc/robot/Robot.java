// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Lights;
import frc.robot.verification.ComponentVerifier;
import frc.robot.verification.ComponentVerifier.VerificationResult.Status;

import java.io.FileReader;
import java.util.HashMap;
import java.util.List;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final HashMap<String, String> busNames = new HashMap<>();

    private final Lights lights;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        lights = new Lights(1, 18);

        //Pull the CAN ID to name pairings from the can.bus file and store them
        try (FileReader reader = new FileReader("src/main/java/frc/robot/verification/can.bus")) {
            StringBuilder builder = new StringBuilder();
            int i;

            while ((i = reader.read()) != -1) builder.append((char) i);
            for (String line : builder.toString().split("\n")) {
                String[] values = line.split("/");

                busNames.put(values[0], values[1].trim());
            }
        } catch (Exception ignored) {}
    }

    @Override
    public void robotInit() {
        verify(ComponentVerifier.verify(m_robotContainer.drivetrain), "Drivetrain");
        verify(List.of(ComponentVerifier.verifyComponent("Battery", "", () -> {
            double voltage = RobotController.getBatteryVoltage();
            if (voltage < 12.2) throw new RuntimeException("Voltage is " + voltage + " V, which is below expected.");
        })), "Battery");
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    private final NetworkTable verification = NetworkTableInstance.getDefault().getTable("Component Verification");
    private void verify(List<ComponentVerifier.VerificationResult> results, String system) {
        Status status = Status.SUCCESS;

        for (ComponentVerifier.VerificationResult result : results) {
            if (result.status() == Status.FAILURE) {
                //Either get the name from the HashMap or just use the provided name
                verification.getEntry(system + "/" + busNames.getOrDefault(result.name(), result.name())).setString(result.message());

                status = Status.FAILURE;
            }
        }

        SmartDashboard.putString("Component Verification", status == Status.FAILURE ? "At least one component had an error." : "No errors were detected.");
        lights.display(status.color);
    }
}
