import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Logger {
    private static Logger INSTANCE;
    private BufferedWriter logWriter;

    private boolean isVerbose;
    private SimpleDateFormat logFormat;

    public Logger() {
        SimpleDateFormat dateFormat = new SimpleDateFormat("dd-MM-yy HH:mm:ss");
        logFormat = new SimpleDateFormat("HH:mm:ss");

        try {
            logWriter = new BufferedWriter(new FileWriter(dateFormat.format(new Date()) + " log.txt", true));
        } catch (IOException e) {
            System.err.println("Unable to open file for logging");
        }

        isVerbose = true;
    }

    public void log(String tag, Object o) {
        String msg = String.format("[%s] %s - %s", logFormat.format(new Date()), tag, o.toString());

        if(isVerbose) {
            System.out.println(msg);
        }

        try {
            logWriter.write(msg + "\r\n");
        } catch (IOException e) {
            System.err.println("Error writing to log");
        }
    }

    public void close() {
        System.out.println("Close");
        try {
            logWriter.flush();
            logWriter.close();
        } catch (IOException e) {
            System.err.println("Unable to flush log writer");
        }

        INSTANCE = null;
    }

    public static Logger getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new Logger();
        }

        return INSTANCE;
    }

    public void setVerbose(boolean isVerbose) {
        this.isVerbose = isVerbose;
    }
}
