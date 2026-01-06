import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.function.Supplier;

import pedroPathing.constants.Constants;
import util.robot;

/**
 * HOW TO USE IT::::
 *
 * PLUG SENSOR INTO I2C
 * then run the ConfigureColorRangefinder TeleOp
 * then remove the sensor and plug it into Digital 0-1, finally run this TeleOp
 */

@TeleOp(name = "ColorSensorTEst")
public class ColorSensorTEst extends OpMode {
    private robot R;
    DigitalChannel pin0;
    DigitalChannel pin1;
    @Override
    public void init() {

    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        telemetry.addData("digital 0", pin0.getState()); // IS PRUPLE
        telemetry.addData("digital 1", pin1.getState()); // IS GREEN
        telemetry.update();
    }

    //private double powerToTicksPerSecond(double power) {
        //return power * ((double) 6000 / 60) * 28;
    }