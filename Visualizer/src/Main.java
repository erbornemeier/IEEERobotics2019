import javax.swing.*;
import java.util.Timer;
import java.util.TimerTask;

public class Main {
    public static final String TITLE = "Robot Viewer";

    public static void main(String[] args) {
        DisplayController displayController = new DisplayController();
        Canvas canvas = new Canvas(displayController);
        JFrame frame = new JFrame(TITLE);

        SwingUtilities.invokeLater(() -> {
            frame.setContentPane(canvas);
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            frame.addWindowListener(new java.awt.event.WindowAdapter() {
                @Override
                public void windowClosing(java.awt.event.WindowEvent windowEvent) {
                    Logger.getInstance().close();
                }
            });

            new Timer().schedule(new TimerTask() {
                @Override
                public void run() {
                    SwingUtilities.invokeLater(() -> {
                        frame.repaint();
                        frame.validate();
                    });
                }
            }, 0, 17);
        });

        try {
            new Server(displayController, frame).run();
        } catch(Exception e) {
            System.out.println(e);
            e.printStackTrace();
        }
    }
}