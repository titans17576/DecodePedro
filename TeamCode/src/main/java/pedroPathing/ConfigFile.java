package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
@Config
public class ConfigFile {
    public static double power = 0.6;

    // Shooter consants
    public static double CONFIGkP = 0.000014;
    public static double CONFIGkI = 0;
    public static double CONFIGkD = 0.00001;

    // High INtake Contasnts
    public static double CONFIGHighkP = 0.000014;
    public static double CONFIGHighkI = 0;
    public static double CONFIGHighkD = 0.00001;

    public static double loopTime = 0.01; //10 msec (used for both PID)
}