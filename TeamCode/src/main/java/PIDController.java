import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

/**
 * Use startController when starting the system,
 */
public class PIDController {

    public double setPoint; // Target value
    public double realValue; // Measured value
    public TimeUnit timeUnit;

    public double kP;
    public double kI;
    public double kD;

    private final ElapsedTime timer = new ElapsedTime();
    private double integralError = 0;
    private double error = 0;

    public PIDController(double kP, double kI, double kD, TimeUnit timeUnit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.timeUnit = timeUnit;
    }

    public void updateSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void startController(double realValue) {
        this.realValue = realValue;
        timer.reset();
    }

    public double update(double realValue) {
        double period = timer.now(timeUnit);
        timer.reset();

        double previousError = error;
        error = setPoint - realValue;

        integralError += period * (setPoint - realValue);

        double derivative = (error - previousError) / period;

        return (kP * error) + (kI * integralError) + (kD * derivative);
    }

}
