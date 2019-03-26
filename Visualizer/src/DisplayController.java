import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class DisplayController {
    private static String TAG  = "DisplayController";

    private ArrayList<Entity> entities;
    private Mothership mothership;
    private Robot robot;
    private HashMap<Character, Block> blocks;
    private Pathfinding pathfinding;

    public DisplayController() {
        initialize();
    }

    private void initialize() {
        entities = new ArrayList<>();
        blocks = new HashMap<>();
        entities.add(new Board());
    }

    @Command("init-robot")
    public void initRobot(String msg) {
        if(robot != null) {
            entities.remove(robot);
        }

        robot = new Robot();
        entities.add(robot);
    }

    @Command("update-robot-pose")
    public void updateRobotPose(String msg) {
        if(robot == null) {
            initRobot("");
        }

        Pattern p = Pattern.compile(
            ".*\\s*x:(\\d*\\.?\\d*)"
            + "\\s*y:(\\d*\\.?\\d*)"
            + "\\s*theta:(-?\\d*\\.?\\d*)"
        );

        Matcher m = p.matcher(msg);

        if(m.matches()) {
            double x = Double.parseDouble(m.group(1));
            double y = Double.parseDouble(m.group(2));
            double thetaDegrees = Double.parseDouble(m.group(3));

            this.robot.updatePose(x, y, thetaDegrees);
        } else {
            Logger.getInstance().log(TAG + ".updateRobotPose()", "Invalid command format: " + msg);
        }
    }

    @Command("init-block")
    public void initBlock(String msg) {
        Pattern p = Pattern.compile(
            ".*\\s*id:(\\d*)"
            + "\\s*x:(\\d*)"
            + "\\s*y:(\\d*)"
        );

        Matcher m = p.matcher(msg);
        if(m.matches()) {
            int id = Integer.parseInt(m.group(1));
            int x = Integer.parseInt(m.group(2));
            int y = Integer.parseInt(m.group(3));

            Block block = new Block(blockIdToLetter(id), x, y);
            blocks.put(block.getLetter(), block);

            this.entities.add(block);
        } else {
            Logger.getInstance().log(TAG + ".initBlock()", "Invalid command format: " + msg);
        }
    }

    @Command("remove-block")
    public void removeBlock(String msg) {
        Pattern p = Pattern.compile(
            ".*\\s*id:(\\d*)"
        );

        Matcher m = p.matcher(msg);
        if(m.matches()) {
            int id = Integer.parseInt(m.group(1));

            Block block = blocks.remove(blockIdToLetter(id));
            if(block != null) {
                entities.remove(block);
            }
        } else {
            Logger.getInstance().log(TAG + ".removeBlock()", "Invalid command format: " + msg);
        }
    }

    @Command("display-mothership")
    public void displayMothership(String msg) {
        if(mothership != null) {
            mothership = null;
            entities.remove(mothership);
        }

        Pattern p = Pattern.compile(
                ".*\\s*x:(\\d*\\.?\\d*)"
                + "\\s*y:(\\d*\\.?\\d*)"
                + "\\s*theta:(-?\\d*\\.?\\d*)\\s*"
                + "\\s*abc_x:(\\d*\\.?\\d*)"
                + "\\s*abc_y:(\\d*\\.?\\d*)"
                + "\\s*af_x:(\\d*\\.?\\d*)"
                + "\\s*af_y:(\\d*\\.?\\d*)"
                + "\\s*def_x:(\\d*\\.?\\d*)"
                + "\\s*def_y:(\\d*\\.?\\d*)"
                + "\\s*cd_x:(\\d*\\.?\\d*)"
                + "\\s*cd_y:(\\d*\\.?\\d*)"
        );

        Matcher m = p.matcher(msg);
        if(m.matches()) {
            double x = Double.parseDouble(m.group(1));
            double y = Double.parseDouble(m.group(2));
            double theta = Double.parseDouble(m.group(3));

            BoardPoint[] points = new BoardPoint[4];
            int group = 4;
            for(int i = 0; i < 4; i++) {
                double pointX = (Double.parseDouble(m.group(group)) * Board.PIXELS_PER_INCH);
                double pointY = (Double.parseDouble(m.group(group + 1)) * Board.PIXELS_PER_INCH);

                points[i] = new BoardPoint(pointX, pointY);
                group += 2;
            }

            mothership = new Mothership(x, y, theta, points);
            entities.add(mothership);
        }
    }

    @Command("init-pathfinding")
    public void createPathfindingGrid(String msg) {
        Pattern p = Pattern.compile(
            ".*\\s*resolution:(\\d*\\.?\\d*)"
            + "\\s*margin:(\\d*\\.?\\d*)"
        );

        Matcher m = p.matcher(msg);
        if(m.matches()) {
            double resolution = Double.parseDouble(m.group(1));
            double margin = Double.parseDouble(m.group(2));

            pathfinding = new Pathfinding(resolution, margin);
        }
    }

    @Command("update-pathfinding-point")
    public void updatePathfindingPoint(String msg) {
        Pattern p = Pattern.compile(
            ".*\\s*x:(\\d*\\.?\\d*)"
            + "\\s*y:(\\d*\\.?\\d*)"
            + "\\s*isBlocked:([tT]rue|[fF]alse)"
        );

        Matcher m = p.matcher(msg);
        if(m.matches()) {
            double x = Double.parseDouble(m.group(1));
            double y = Double.parseDouble(m.group(2));
            boolean isBlocked = Boolean.parseBoolean(m.group(3).toLowerCase());

            pathfinding.updatePoint(x, y, isBlocked);
        }
    }

    public char blockIdToLetter(int id) {
        return (char) ('A' + id);
    }

    public ArrayList<Entity> getEntities() {
        return entities;
    }

    public void reset() {
        robot = null;

        initialize();
    }

    public Pathfinding getPathfinding() {
        return pathfinding;
    }
}
