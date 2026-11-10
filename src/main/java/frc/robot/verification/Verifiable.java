package frc.robot.verification;

import edu.wpi.first.util.sendable.Sendable;

import java.util.List;

public interface Verifiable {
    List<Sendable> components();
}
