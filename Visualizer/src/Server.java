import javax.swing.*;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.HashMap;

public class Server extends Thread {
    private static final String TAG = "Server";

    private static int port = 4445;

    private DisplayController displayController;
    private JFrame frame;

    private DatagramSocket socket;
    private boolean running;
    private byte[] buf = new byte[1024];

    private HashMap<String, Method> commands;

    public Server(DisplayController displayController, JFrame frame) throws SocketException {
        this.displayController = displayController;
        this.frame = frame;

        initCommands();

        socket = new DatagramSocket(port);
        System.out.println("Server Listening on Port: " + port);
    }

    private void initCommands() {
        this.commands = new HashMap<>();

        for(Method method : DisplayController.class.getMethods()) {
            if(method.isAnnotationPresent(Command.class)) {
                method.setAccessible(true);
                commands.put(method.getAnnotation(Command.class).value(), method);
            }
        }
    }

    @Override
    public void run() {
        running = true;

        while (running) {
            DatagramPacket packet = new DatagramPacket(buf, buf.length);

            try {
                socket.receive(packet);
            } catch (IOException e) {
                Logger.getInstance().log(TAG, "Error receiving packet: " + e);
            }

            String data = new String(packet.getData(), 0, packet.getLength());
            Logger.getInstance().log(TAG, "Received message: " + data);

            if(data.equals("initialize")) {
                // TODO: Get working, but not really important
//                InetAddress address = packet.getAddress();
//                int port = packet.getPort();
////                byte[] response = "success".getBytes();
//                packet = new DatagramPacket(packet.getData(), packet.getData().length, address, port);
//
//                try {
//                    socket.send(packet);
//                } catch (IOException e) {
//                    Logger.getInstance().log(TAG, "Error accepting initialize");
//                }
//                return;
            } else if (data.equals("end")) {
                System.out.println("Closing Server");
                running = false;
            }

            boolean matchedCommand = false;
            for(String command : commands.keySet()) {
                if(data.startsWith(command)) {
                    try {
                        commands.get(command).invoke(displayController, data);
                        matchedCommand = true;
                        break;
                    } catch (IllegalAccessException e) {
                        System.err.println("Command method is private");
                        e.printStackTrace();
                    } catch (InvocationTargetException e) {
                        System.err.println("Invalid invocation");
                        e.printStackTrace();
                    }
                }
            }

            if(!matchedCommand) {
                Logger.getInstance().log(TAG, "Message did not match any commands: " + data);
            }
        }
        socket.close();
    }

    public void draw() {
        SwingUtilities.invokeLater(() -> {
            this.frame.repaint();
            this.frame.validate();
        });
    }
}


// Respond
//            InetAddress address = packet.getAddress();
//            int port = packet.getPort();
//            packet = new DatagramPacket(buf, buf.length, address, port);
//            String received = new String(packet.getData(), 0, packet.getLength());
//            socket.send(packet);

