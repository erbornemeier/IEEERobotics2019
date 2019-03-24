import java.awt.*;

public class Block extends Entity {
    private static final BasicStroke STROKE = new BasicStroke(2);

    private char letter;
    private double x;
    private double y;

    private int drawSize;

    /**
     *
     * @param letter The block letter
     * @param jsonX The integer x from the json file
     * @param jsonY The integer y from the json file
     */
    public Block(char letter, int jsonX, int jsonY) {
        this.letter = letter;

        drawSize = (int) (2 * Board.PIXELS_PER_INCH);
        this.x = (jsonX * Board.PIXELS_PER_FOOT);
        this.y = (jsonY * Board.PIXELS_PER_FOOT);
    }

    @Override
    public void draw(Graphics2D g) {
        g.setColor(Color.BLACK);
        g.setStroke(STROKE);

        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_OFF);
        int drawX = (int) (x + Board.MARGIN + (Board.PIXELS_PER_FOOT / 2) - (drawSize / 2.0));
        int drawY = Canvas.CANVAS_HEIGHT - (int)(y + Board.MARGIN + (Board.PIXELS_PER_FOOT / 2) + (drawSize / 2.0));
        g.drawRect(drawX, drawY, drawSize, drawSize);

        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        g.drawString("" + letter, drawX + drawSize/2 - 4, drawY + drawSize/2 + 5);
    }

    public char getLetter() {
        return letter;
    }

    @Override
    public boolean equals(Object o) {
        if(!(o instanceof Block)) {
            return false;
        }

        return letter == ((Block) o).letter;
    }
}
