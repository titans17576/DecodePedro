import com.qualcomm.robotcore.util.ElapsedTime;

// This is a positional PID
public class PIDController {
    double kP, kI, kD;
    double lastError = 0;
    double integralSum = 0;
    public ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }

    public double calculate(double target, double current) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        if (dt == 0) dt = 0.01;

        double p = kP * error;

        integralSum += error * dt;
        double i = kI * integralSum;

        double derivative = (error - lastError) / dt;
        double d = kD * derivative;

        lastError = error;

        return p + i + d;
    }

    public void reset() {
        timer.reset();
        lastError = 0;
        integralSum = 0;
    }
}
