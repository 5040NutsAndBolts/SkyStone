package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;

public class PwmControlTest extends PWMOutputImpl {
    /**
     * Constructor
     *
     * @param controller Digital port controller this port is attached to
     * @param port       port on the digital port controller
     */
    public PwmControlTest(PWMOutputController controller, int port) {
        super(controller, port);
    }
}
