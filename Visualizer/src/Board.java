import java.awt.*;

public class Board extends Entity {
    public static int MARGIN = 10; // pixels
    public static int LENGTH_PIXELS = Canvas.CANVAS_HEIGHT - (MARGIN * 2);
    public static int LENGTH_FEET = 8;

    public static double PIXELS_PER_INCH = (double)LENGTH_PIXELS / (LENGTH_FEET * 12);
    public static double PIXELS_PER_FOOT = (double)LENGTH_PIXELS / LENGTH_FEET;

    private BasicStroke outerStroke;
    private BasicStroke gridStroke;

    public Board() {
        outerStroke = new BasicStroke(3);
        gridStroke = new BasicStroke(1);
    }

    @Override
    public void draw(Graphics2D g) {
        g.setStroke(gridStroke);
        g.setColor(Color.LIGHT_GRAY);
        for(int i = 0; i < LENGTH_FEET; i++) {
            for(int j = 0; j < LENGTH_FEET; j++) {
                g.drawRect((int) (i * PIXELS_PER_FOOT + MARGIN),
                        (int) (j * PIXELS_PER_FOOT + MARGIN),
                        (int) PIXELS_PER_FOOT,
                        (int) PIXELS_PER_FOOT);
            }
        }

        g.setStroke(outerStroke);
        g.setColor(Color.BLACK);
        g.drawRect(MARGIN, MARGIN, Canvas.CANVAS_HEIGHT - (MARGIN * 2), Canvas.CANVAS_HEIGHT - (MARGIN * 2));
    }
}
