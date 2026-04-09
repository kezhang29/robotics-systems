import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;
import javax.swing.*;

public class PurePursuit extends JPanel implements KeyListener, MouseListener {

    // -------------------------------------------------------------------------
    // Tuning knobs
    // -------------------------------------------------------------------------
    private static final double LOOK_AHEAD  = 40.0;   // px
    private static final double SPEED       = 3.0;    // px per frame
    private static final double KP_TURN     = 0.08;   // heading proportional gain
    private static final double FINISH_DIST = SPEED + 3;

    // -------------------------------------------------------------------------
    // Path & robot state
    // -------------------------------------------------------------------------
    private final List<Point2D.Double> waypoints = new ArrayList<>();
    private final List<Point2D.Double> trajectory = new ArrayList<>();

    private double robotX, robotY, robotHeading;
    private int    lastFoundIndex;
    private Point2D.Double goalPt;

    private boolean running  = false;
    private boolean finished = false;

    private Timer animTimer;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
    public PurePursuit() {
        setBackground(Color.WHITE);
        setPreferredSize(new Dimension(900, 650));
        setFocusable(true);
        addKeyListener(this);
        addMouseListener(this);

        animTimer = new Timer(16, e -> {
            if (running) {
                step();
                repaint();
            }
        });
    }

    // -------------------------------------------------------------------------
    // Pure Pursuit core
    // -------------------------------------------------------------------------
    private void step() {
        if (waypoints.size() < 2 || finished) return;

        Point2D.Double end = waypoints.get(waypoints.size() - 1);
        if (dist(robotX, robotY, end.x, end.y) <= FINISH_DIST) {
            finished = true;
            running  = false;
            animTimer.stop();
            repaint();
            return;
        }

        goalPt = findGoalPoint();
        if (goalPt == null) return;

        double absTarget = Math.toDegrees(Math.atan2(goalPt.y - robotY, goalPt.x - robotX));
        if (absTarget < 0) absTarget += 360;

        double turnErr = absTarget - robotHeading;
        if (turnErr >  180) turnErr -= 360;
        if (turnErr < -180) turnErr += 360;

        robotHeading += KP_TURN * turnErr;
        robotHeading  = ((robotHeading % 360) + 360) % 360;

        robotX += SPEED * Math.cos(Math.toRadians(robotHeading));
        robotY += SPEED * Math.sin(Math.toRadians(robotHeading));

        trajectory.add(new Point2D.Double(robotX, robotY));
    }

    private Point2D.Double findGoalPoint() {
        double cx = robotX, cy = robotY, ld = LOOK_AHEAD;

        for (int i = lastFoundIndex; i < waypoints.size() - 1; i++) {
            double x1 = waypoints.get(i).x   - cx;
            double y1 = waypoints.get(i).y   - cy;
            double x2 = waypoints.get(i+1).x - cx;
            double y2 = waypoints.get(i+1).y - cy;

            double dx = x2 - x1, dy = y2 - y1;
            double dr = Math.sqrt(dx*dx + dy*dy);
            if (dr == 0) continue;

            double D    = x1*y2 - x2*y1;
            double disc = ld*ld * dr*dr - D*D;
            if (disc < 0) continue;

            double sq   = Math.sqrt(disc);
            double dr2  = dr * dr;

            double sx1 = (D*dy + sgn(dy)*dx*sq) / dr2 + cx;
            double sx2 = (D*dy - sgn(dy)*dx*sq) / dr2 + cx;
            double sy1 = (-D*dx + Math.abs(dy)*sq) / dr2 + cy;
            double sy2 = (-D*dx - Math.abs(dy)*sq) / dr2 + cy;

            double minX = Math.min(waypoints.get(i).x, waypoints.get(i+1).x);
            double maxX = Math.max(waypoints.get(i).x, waypoints.get(i+1).x);
            double minY = Math.min(waypoints.get(i).y, waypoints.get(i+1).y);
            double maxY = Math.max(waypoints.get(i).y, waypoints.get(i+1).y);
            double eps  = 1e-6;

            boolean v1 = inRange(sx1, minX, maxX, eps) && inRange(sy1, minY, maxY, eps);
            boolean v2 = inRange(sx2, minX, maxX, eps) && inRange(sy2, minY, maxY, eps);
            if (!v1 && !v2) continue;

            Point2D.Double pt;
            if (v1 && v2) {
                double d1 = dist(sx1, sy1, waypoints.get(i+1).x, waypoints.get(i+1).y);
                double d2 = dist(sx2, sy2, waypoints.get(i+1).x, waypoints.get(i+1).y);
                pt = (d1 < d2) ? new Point2D.Double(sx1, sy1) : new Point2D.Double(sx2, sy2);
            } else {
                pt = v1 ? new Point2D.Double(sx1, sy1) : new Point2D.Double(sx2, sy2);
            }

            double dGoal  = dist(pt.x, pt.y, waypoints.get(i+1).x, waypoints.get(i+1).y);
            double dRobot = dist(cx,   cy,   waypoints.get(i+1).x, waypoints.get(i+1).y);

            if (dGoal < dRobot) {
                lastFoundIndex = i;
                return pt;
            } else {
                lastFoundIndex = i + 1;
            }
        }

        if (lastFoundIndex < waypoints.size())
            return new Point2D.Double(waypoints.get(lastFoundIndex).x,
                                      waypoints.get(lastFoundIndex).y);
        return null;
    }

    // -------------------------------------------------------------------------
    // Painting
    // -------------------------------------------------------------------------
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawGrid(g2);

        if (waypoints.isEmpty()) {
            g2.setColor(new Color(160, 160, 160));
            g2.setFont(new Font("SansSerif", Font.PLAIN, 15));
            FontMetrics fm = g2.getFontMetrics();
            String msg = "Left-click to place waypoints, then press Space to run";
            g2.drawString(msg, (getWidth() - fm.stringWidth(msg)) / 2, getHeight() / 2);
            return;
        }

        drawPath(g2);
        drawTrajectory(g2);

        if (running && goalPt != null) {
            drawLookAhead(g2);
            drawGoalLine(g2);
        }

        drawWaypoints(g2);
        drawRobot(g2);
        drawHUD(g2);
    }

    private void drawGrid(Graphics2D g2) {
        g2.setColor(new Color(235, 235, 235));
        g2.setStroke(new BasicStroke(0.5f));
        for (int x = 0; x < getWidth(); x += 40)
            g2.drawLine(x, 0, x, getHeight());
        for (int y = 0; y < getHeight(); y += 40)
            g2.drawLine(0, y, getWidth(), y);
    }

    private void drawPath(Graphics2D g2) {
        if (waypoints.size() < 2) return;
        g2.setColor(new Color(55, 138, 221, 80));
        g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                1f, new float[]{8f, 5f}, 0f));
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Point2D.Double a = waypoints.get(i), b = waypoints.get(i + 1);
            g2.drawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y);
        }
        g2.setStroke(new BasicStroke(1.5f));
    }

    private void drawTrajectory(Graphics2D g2) {
        if (trajectory.size() < 2) return;
        g2.setColor(new Color(127, 119, 221, 180));
        g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        for (int i = 0; i < trajectory.size() - 1; i++) {
            Point2D.Double a = trajectory.get(i), b = trajectory.get(i + 1);
            g2.drawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y);
        }
        g2.setStroke(new BasicStroke(1.5f));
    }

    private void drawLookAhead(Graphics2D g2) {
        int r = (int) LOOK_AHEAD;
        g2.setColor(new Color(239, 159, 39, 60));
        g2.fillOval((int)(robotX - r), (int)(robotY - r), r*2, r*2);
        g2.setColor(new Color(239, 159, 39, 160));
        g2.setStroke(new BasicStroke(1f));
        g2.drawOval((int)(robotX - r), (int)(robotY - r), r*2, r*2);
    }

    private void drawGoalLine(Graphics2D g2) {
        if (goalPt == null) return;
        g2.setColor(new Color(226, 75, 74, 150));
        g2.setStroke(new BasicStroke(1.5f));
        g2.drawLine((int)robotX, (int)robotY, (int)goalPt.x, (int)goalPt.y);
        g2.setColor(new Color(226, 75, 74));
        g2.fillOval((int)goalPt.x - 5, (int)goalPt.y - 5, 10, 10);
    }

    private void drawWaypoints(Graphics2D g2) {
        for (int i = 0; i < waypoints.size(); i++) {
            Point2D.Double wp = waypoints.get(i);
            boolean isStart = (i == 0);
            boolean isEnd   = (i == waypoints.size() - 1);

            g2.setColor(isStart ? new Color(99, 153, 34)
                      : isEnd   ? new Color(226, 75, 74)
                                : new Color(55, 138, 221));
            g2.fillOval((int)wp.x - 6, (int)wp.y - 6, 12, 12);

            g2.setColor(Color.WHITE);
            g2.setStroke(new BasicStroke(1.5f));
            g2.drawOval((int)wp.x - 6, (int)wp.y - 6, 12, 12);

            g2.setColor(new Color(60, 60, 60));
            g2.setFont(new Font("SansSerif", Font.PLAIN, 11));
            g2.drawString(String.valueOf(i + 1), (int)wp.x + 9, (int)wp.y - 5);
        }
    }

    private void drawRobot(Graphics2D g2) {
        if (waypoints.isEmpty() && !running && !finished) return;

        AffineTransform old = g2.getTransform();
        g2.translate(robotX, robotY);
        g2.rotate(Math.toRadians(robotHeading));

        int[] xs = { 12, -8, -8 };
        int[] ys = {  0, -7,  7 };
        g2.setColor(new Color(99, 153, 34));
        g2.fillPolygon(xs, ys, 3);
        g2.setColor(new Color(59, 109, 17));
        g2.setStroke(new BasicStroke(1.5f));
        g2.drawPolygon(xs, ys, 3);

        g2.setTransform(old);
    }

    private void drawHUD(Graphics2D g2) {
        String state = finished ? "Done — press C to clear"
                     : running  ? "Running — Space to pause"
                     : waypoints.size() < 2 ? "Add at least 2 waypoints"
                     : "Space to run  ·  U to undo  ·  C to clear";

        g2.setColor(new Color(0, 0, 0, 120));
        g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
        FontMetrics fm = g2.getFontMetrics();
        int sw = fm.stringWidth(state);
        g2.fillRoundRect(getWidth() - sw - 22, 10, sw + 16, 24, 8, 8);
        g2.setColor(Color.WHITE);
        g2.drawString(state, getWidth() - sw - 14, 27);
    }

    // -------------------------------------------------------------------------
    // Sim control
    // -------------------------------------------------------------------------
    private void startSim() {
        if (waypoints.size() < 2) return;
        robotX       = waypoints.get(0).x;
        robotY       = waypoints.get(0).y;
        robotHeading = Math.toDegrees(Math.atan2(
                waypoints.get(1).y - waypoints.get(0).y,
                waypoints.get(1).x - waypoints.get(0).x));
        if (robotHeading < 0) robotHeading += 360;
        lastFoundIndex = 0;
        trajectory.clear();
        goalPt   = null;
        finished = false;
        running  = true;
        animTimer.start();
    }

    private void pauseSim() {
        running = false;
        animTimer.stop();
        repaint();
    }

    private void clearAll() {
        pauseSim();
        waypoints.clear();
        trajectory.clear();
        goalPt   = null;
        finished = false;
        repaint();
    }

    // -------------------------------------------------------------------------
    // Input handlers
    // -------------------------------------------------------------------------
    @Override public void mousePressed(MouseEvent e) {
        if (e.getButton() == MouseEvent.BUTTON1) {
            waypoints.add(new Point2D.Double(e.getX(), e.getY()));
            if (!running) {
                robotX = waypoints.get(0).x;
                robotY = waypoints.get(0).y;
                robotHeading = 0;
            }
            repaint();
        }
    }

    @Override public void keyPressed(KeyEvent e) {
        switch (e.getKeyCode()) {
            case KeyEvent.VK_SPACE:
                if (finished)       { startSim(); }
                else if (running)   { pauseSim(); }
                else                { startSim(); }
                break;
            case KeyEvent.VK_U:
                if (!waypoints.isEmpty() && !running) {
                    waypoints.remove(waypoints.size() - 1);
                    repaint();
                }
                break;
            case KeyEvent.VK_C:
                clearAll();
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Utilities
    // -------------------------------------------------------------------------
    private static double dist(double x1, double y1, double x2, double y2) {
        double dx = x2-x1, dy = y2-y1;
        return Math.sqrt(dx*dx + dy*dy);
    }
    private static double sgn(double n)  { return n >= 0 ? 1.0 : -1.0; }
    private static boolean inRange(double v, double lo, double hi, double eps) {
        return v >= lo - eps && v <= hi + eps;
    }

    // -------------------------------------------------------------------------
    // Unused interface methods
    // -------------------------------------------------------------------------
    @Override public void keyReleased(KeyEvent e) {}
    @Override public void keyTyped(KeyEvent e) {}
    @Override public void mouseClicked(MouseEvent e) {}
    @Override public void mouseReleased(MouseEvent e) {}
    @Override public void mouseEntered(MouseEvent e) {}
    @Override public void mouseExited(MouseEvent e) {}

    // -------------------------------------------------------------------------
    // Entry point
    // -------------------------------------------------------------------------
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Pure Pursuit");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            PurePursuit canvas = new PurePursuit();
            frame.add(canvas);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
            canvas.requestFocusInWindow();
        });
    }
}