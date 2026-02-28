package drive;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import javax.swing.*;
import odom.Odom;
import pid.PID;

public class DrivetopointSim extends JPanel {

    // robot 
    private static class Robot {
        double x, y, heading;
        double leftEncoder = 0, rightEncoder = 0;
        static final double TRACK_WIDTH = 30.0;
        static final double V_TO_SPEED  = 8.0;
        static final double MAX_VOLTAGE = 12.0;

        Robot(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }

        void driveWithVoltage(double leftV, double rightV, double dt) {
            leftV  = DriveUtil.clamp(leftV,  -MAX_VOLTAGE, MAX_VOLTAGE);
            rightV = DriveUtil.clamp(rightV, -MAX_VOLTAGE, MAX_VOLTAGE);
            double leftSpd  = leftV  * V_TO_SPEED;
            double rightSpd = rightV * V_TO_SPEED;
            double linear   = (leftSpd + rightSpd) / 2.0;
            double angular  = (rightSpd - leftSpd) / TRACK_WIDTH;
            heading += Math.toDegrees(angular) * dt;
            double rad = Math.toRadians(heading);
            x += linear * Math.sin(rad) * dt;
            y += linear * Math.cos(rad) * dt;
            leftEncoder  += leftSpd  * dt;
            rightEncoder += rightSpd * dt;
        }

        double forwardEncoder()  { return (leftEncoder + rightEncoder) / 2.0; }
        double sidewaysEncoder() { return 0; }
    }

    // pid
    static final double DRIVE_KP = 0.4,  DRIVE_KI = 0,  DRIVE_KD = 0.5;
    static final double HEAD_KP  = 0.08, HEAD_KI  = 0,  HEAD_KD  = 0.1;
    static final double DRIVE_MAX_V    = 8.0;
    static final double DRIVE_MIN_V    = 0.0;
    static final double HEAD_MAX_V     = 3.0;
    static final double DRIVE_SETTLE_E = 3.0;
    static final double DRIVE_SETTLE_T = 0.3;
    static final double DRIVE_TIMEOUT  = 5.0;

    //state
    Robot robot = new Robot(0, 0, 0);
    Odom  odom  = new Odom(5, 0);

    double targetX = 80, targetY = 80;
    PID    drivePID, headPID;

    ArrayList<Point2D.Double> trailTrue = new ArrayList<>();
    ArrayList<Point2D.Double> trailOdom = new ArrayList<>();

    boolean running = false, settled = false, linePrev = false;
    double  startAngle = 0, elapsed = 0;

    enum Drag { NONE, ROBOT, TARGET }
    Drag drag = Drag.NONE;

    static final double SCALE = 3.5;

    static final Color BG       = Color.WHITE;
    static final Color GRID     = new Color(220, 225, 230);
    static final Color AXIS     = new Color(30,  30,  30);
    static final Color C_ROBOT  = new Color(0,   120, 200);
    static final Color C_FWD    = new Color(0,   180, 80);
    static final Color C_TARGET = new Color(210, 40,  40);
    static final Color C_TRUE   = new Color(210, 40,  40);
    static final Color C_ODOM   = new Color(0,   120, 200);
    static final Color DIM      = new Color(160, 160, 160);
    static final Color BLACK    = Color.BLACK;

    Font fontS;

    // ── Constructor ───────────────────────────────────────────────────────────
    public DrivetopointSim() {
        setPreferredSize(new Dimension(860, 700));
        setBackground(BG);
        fontS = new Font("Courier New", Font.PLAIN, 12);
        setFocusable(true);

        addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                requestFocusInWindow();
                Point rp = toScreen(robot.x, robot.y);
                Point tp = toScreen(targetX, targetY);
                if (Math.hypot(e.getX() - rp.x, e.getY() - rp.y) < 16) {
                    drag = Drag.ROBOT; running = false;
                } else if (Math.hypot(e.getX() - tp.x, e.getY() - tp.y) < 16) {
                    drag = Drag.TARGET; running = false;
                }
            }
            public void mouseReleased(MouseEvent e) {
                if (drag != Drag.NONE) prepareSim();
                drag = Drag.NONE;
                repaint();
            }
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            public void mouseDragged(MouseEvent e) {
                Point2D.Double f = toField(e.getX(), e.getY());
                if (drag == Drag.ROBOT) {
                    robot.x = f.x; robot.y = f.y;
                } else if (drag == Drag.TARGET) {
                    targetX = f.x; targetY = f.y;
                }
                repaint();
            }
        });

        addKeyListener(new KeyAdapter() {
            public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_SPACE) {
                    if (settled) {
                        prepareSim();
                    } else {
                        running = !running;
                    }
                    repaint();
                }
            }
        });

        prepareSim();

        new Timer(16, e -> {
            if (running && !settled) { stepSim(0.016); repaint(); }
        }).start();
    }

    // sim
    void prepareSim() {
        robot.leftEncoder  = 0;
        robot.rightEncoder = 0;
        odom.setPosition(robot.x, robot.y, robot.heading);

        trailTrue.clear(); trailTrue.add(new Point2D.Double(robot.x, robot.y));
        trailOdom.clear(); trailOdom.add(new Point2D.Double(odom.xPosition, odom.yPosition));

        double dist = Math.hypot(targetX - robot.x, targetY - robot.y);
        startAngle  = Math.toDegrees(Math.atan2(targetX - robot.x, targetY - robot.y));

        drivePID = new PID(dist, DRIVE_KP, DRIVE_KI, DRIVE_KD, 0,
                           DRIVE_SETTLE_E, DRIVE_SETTLE_T, DRIVE_TIMEOUT);
        headPID  = new PID(DriveUtil.reduceNegative180to180(startAngle - robot.heading),
                           HEAD_KP, HEAD_KI, HEAD_KD, 0, 0, 0, 0);

        running = false; settled = false; linePrev = false; elapsed = 0;
    }

    void stepSim(double dt) {
       
        odom.updatePosition(robot.forwardEncoder(), robot.sidewaysEncoder(), robot.heading);

        double ox = odom.xPosition, oy = odom.yPosition;

        boolean lineNow = DriveUtil.isLineSettled(targetX, targetY, startAngle, ox, oy);
        if (lineNow && !linePrev) { settled = true; running = false; return; }
        linePrev = lineNow;

        double driveError   = Math.hypot(targetX - ox, targetY - oy);
        double rawHeading   = Math.toDegrees(Math.atan2(targetX - ox, targetY - oy));
        double headingError = DriveUtil.reduceNegative180to180(rawHeading - robot.heading);

        double driveOut = drivePID.calculate(driveError);
        double scale    = Math.cos(Math.toRadians(headingError));
        driveOut       *= scale;

        double headOut = headPID.calculate(DriveUtil.reduceNegative90to90(headingError));
        if (driveError < DRIVE_SETTLE_E) headOut = 0;

        driveOut = DriveUtil.clamp(driveOut, -Math.abs(scale) * DRIVE_MAX_V, Math.abs(scale) * DRIVE_MAX_V);
        headOut  = DriveUtil.clamp(headOut, -HEAD_MAX_V, HEAD_MAX_V);
        driveOut = DriveUtil.clampMinVoltage(driveOut, DRIVE_MIN_V);

        // 3. Move true robot + advance encoders
        robot.driveWithVoltage(DriveUtil.leftVoltage(driveOut, headOut),
                               DriveUtil.rightVoltage(driveOut, headOut), dt);
        elapsed += dt;
        if (drivePID.isSettled()) { settled = true; running = false; }

        // 4. Record both trails
        trailTrue.add(new Point2D.Double(robot.x, robot.y));
        trailOdom.add(new Point2D.Double(odom.xPosition, odom.yPosition));
        if (trailTrue.size() > 6000) trailTrue.remove(0);
        if (trailOdom.size() > 6000) trailOdom.remove(0);
    }

    // coords
    int cx() { return getWidth()  / 2; }
    int cy() { return getHeight() / 2; }

    Point toScreen(double fx, double fy) {
        return new Point((int)(cx() + fx * SCALE), (int)(cy() - fy * SCALE));
    }

    Point2D.Double toField(int sx, int sy) {
        return new Point2D.Double((sx - cx()) / SCALE, -(sy - cy()) / SCALE);
    }

    // paint
    @Override
    protected void paintComponent(Graphics g0) {
        super.paintComponent(g0);
        Graphics2D g = (Graphics2D) g0;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,      RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        int w = getWidth(), h = getHeight();
        g.setColor(BG); g.fillRect(0, 0, w, h);

        // grid
        int step = (int)(24 * SCALE);
        g.setColor(GRID);
        for (int gx = cx() % step; gx < w; gx += step) g.drawLine(gx, 0, gx, h);
        for (int gx = cx();        gx >= 0; gx -= step) g.drawLine(gx, 0, gx, h);
        for (int gy = cy() % step; gy < h; gy += step)  g.drawLine(0, gy, w, gy);
        for (int gy = cy();        gy >= 0; gy -= step)  g.drawLine(0, gy, w, gy);

        // axes
        g.setColor(AXIS); g.setStroke(new BasicStroke(1.5f));
        g.drawLine(cx(), 0, cx(), h);
        g.drawLine(0, cy(), w, cy());
        g.setStroke(new BasicStroke(1f));
        g.setFont(fontS); g.setColor(BLACK);
        g.drawString("+Y", cx() + 4, 14);
        g.drawString("+X", w - 24,   cy() + 14);

        // true path
        float[] dash = {6f, 4f};
        g.setStroke(new BasicStroke(1.8f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, dash, 0));
        for (int i = 1; i < trailTrue.size(); i++) {
            float frac = (float) i / trailTrue.size();
            g.setColor(new Color(210, 40, 40, (int)(60 + 160 * frac)));
            Point p0 = toScreen(trailTrue.get(i-1).x, trailTrue.get(i-1).y);
            Point p1 = toScreen(trailTrue.get(i).x,   trailTrue.get(i).y);
            g.drawLine(p0.x, p0.y, p1.x, p1.y);
        }

        // odom path
        g.setStroke(new BasicStroke(2f));
        for (int i = 1; i < trailOdom.size(); i++) {
            float frac = (float) i / trailOdom.size();
            g.setColor(new Color(0, 120, 200, (int)(60 + 160 * frac)));
            Point p0 = toScreen(trailOdom.get(i-1).x, trailOdom.get(i-1).y);
            Point p1 = toScreen(trailOdom.get(i).x,   trailOdom.get(i).y);
            g.drawLine(p0.x, p0.y, p1.x, p1.y);
        }
        g.setStroke(new BasicStroke(1f));

        // target
        Point ts = toScreen(targetX, targetY);
        g.setColor(C_TARGET); g.setStroke(new BasicStroke(2f));
        g.drawOval(ts.x - 10, ts.y - 10, 20, 20);
        g.drawLine(ts.x - 8, ts.y, ts.x + 8, ts.y);
        g.drawLine(ts.x, ts.y - 8, ts.x, ts.y + 8);
        g.setStroke(new BasicStroke(1f));

        // true pos
        Point  rs  = toScreen(robot.x, robot.y);
        int    sz  = 12;
        double rad = Math.toRadians(robot.heading);
        double[] bx = {-sz, sz, sz, -sz};
        double[] by = {-sz,-sz, sz,  sz};
        int[] px = new int[4], py = new int[4];
        for (int i = 0; i < 4; i++) {
            px[i] = (int)(rs.x + bx[i] * Math.cos(rad) - by[i] * Math.sin(rad));
            py[i] = (int)(rs.y + bx[i] * Math.sin(rad) + by[i] * Math.cos(rad));
        }
        g.setColor(new Color(220, 235, 255)); g.fillPolygon(px, py, 4);
        g.setColor(C_ROBOT); g.setStroke(new BasicStroke(2f)); g.drawPolygon(px, py, 4);
        int fwdX = (int)(rs.x + sz * 1.5 * Math.sin(rad));
        int fwdY = (int)(rs.y - sz * 1.5 * Math.cos(rad));
        g.setColor(C_FWD); g.drawLine(rs.x, rs.y, fwdX, fwdY);

        // odom ghost
        Point os = toScreen(odom.xPosition, odom.yPosition);
        g.setColor(new Color(0, 120, 200, 160));
        g.setStroke(new BasicStroke(1.5f));
        g.drawOval(os.x - 5, os.y - 5, 10, 10);
        g.setStroke(new BasicStroke(1f));

        // legend
        int lx = w - 155, ly = 16;
        g.setFont(fontS);
        g.setStroke(new BasicStroke(1.8f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, dash, 0));
        g.setColor(C_TRUE); g.drawLine(lx, ly - 3, lx + 18, ly - 3);
        g.setStroke(new BasicStroke(1f)); g.drawString("true path", lx + 24, ly);
        ly += 18;
        g.setStroke(new BasicStroke(2f));
        g.setColor(C_ODOM); g.drawLine(lx, ly - 3, lx + 18, ly - 3);
        g.setStroke(new BasicStroke(1f)); g.drawString("odom path", lx + 24, ly);

        // prints
        g.setColor(BLACK);
        g.drawString(String.format("true  x: %.1f  y: %.1f", robot.x, robot.y), 10, 18);
        g.drawString(String.format("odom  x: %.1f  y: %.1f", odom.xPosition, odom.yPosition), 10, 34);

        // status
        g.setColor(DIM);
        g.drawString("drag robot or target  |  space to start/stop", 10, h - 10);
        String status = settled ? "SETTLED" : running ? "RUNNING" : "PAUSED";
        Color  sc     = settled ? new Color(0,160,60) : running ? new Color(200,130,0) : DIM;
        g.setColor(sc);
        g.drawString(status, w - 72, h - 10);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Drive-to-Point");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            DrivetopointSim sim = new DrivetopointSim();
            frame.add(sim);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
            sim.requestFocusInWindow();
        });
    }
}