import sun.rmi.runtime.Log;

import javax.swing.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.*;
import java.util.HashMap;

public class Server extends Thread {
    private static final String TAG = "Server";

    private static int port = 4445;

    private DisplayController displayController;
    private JFrame frame;

    private ServerSocket socket;
    private boolean running;
    private byte[] buf = new byte[1024];

    private HashMap<String, Method> commands;

    private BufferedReader bufferedReader;

    public Server(DisplayController displayController, JFrame frame) throws IOException {
        this.displayController = displayController;
        this.frame = frame;

        initCommands();

        socket = new ServerSocket(port);
        System.out.println("Server Listening on Port: " + port);

        Socket connectionSocket = socket.accept();
        bufferedReader = new BufferedReader(new InputStreamReader(connectionSocket.getInputStream()));
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
//            DatagramPacket packet = new DatagramPacket(buf, buf.length);
//
//            try {
//                socket.receive(packet);
//            } catch (IOException e) {
//                Logger.getInstance().log(TAG, "Error receiving packet: " + e);
//            }

            String data = "";
            try {
                data = bufferedReader.readLine();
            } catch (IOException e) {
                Logger.getInstance().log(TAG, "Error receiving packet: " + e);
            }

//            String data = new String(packet.getData(), 0, packet.getLength());
//            Logger.getInstance().log(TAG, "Received message: " + data);

            if(data.equals("initialize")) {
                // TODO: Get working, but not really important
                // TODO: Probably not necessary anymore
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
                    final String cmd = data;
                    SwingUtilities.invokeLater(() -> {
                        try {
                            commands.get(command).invoke(displayController, cmd);
                        } catch (IllegalAccessException e) {
                            System.err.println("Command method is private");
                            e.printStackTrace();
                        } catch (InvocationTargetException e) {
                            System.err.println("Invalid invocation");
                            e.printStackTrace();
                        }
                    });

                    matchedCommand = true;
                    break;
                }
            }

            if(!matchedCommand) {
                Logger.getInstance().log(TAG, "Message did not match any commands: " + data);
            }
        }

        try {
            socket.close();
        } catch (IOException e) {
            Logger.getInstance().log(TAG, "Error closing: " + e);
        }
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

