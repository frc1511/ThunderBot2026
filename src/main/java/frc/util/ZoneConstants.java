package frc.util;

import java.util.LinkedHashSet;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ZoneConstants {
    public static final Translation2d kRightBlueTrenchCenterSidePoint = new Translation2d(Units.Meters.zero(), Units.Meters.of(4.624));
    public static final Distance kTrenchWidth = Units.Meters.of(1.427);
    public static final Distance kDepth = Units.Meters.of(0.5715);
    public static final Distance kBumpWidth = Units.Meters.of(2.60);

    public static Rectangle2d constructZone(boolean isBump, boolean isBlueSide, boolean isDSside, boolean isRightSide) {
        Translation2d rightPoint = kRightBlueTrenchCenterSidePoint;
        Translation2d leftPoint = kRightBlueTrenchCenterSidePoint;

        // X dimension
        if (!isBlueSide) {
            isRightSide = !isRightSide;
        }

        if (isBump) {
            Translation2d bumpOffset = new Translation2d(
                kTrenchWidth
                .plus(
                    !isRightSide ? kBumpWidth // If on the left, add the bump so that the offset is the sum of the right side, and the right corner is in the hub 
                    : Units.Meters.zero()
                ), Units.Meters.zero());

            rightPoint = rightPoint.plus(bumpOffset);
            leftPoint = leftPoint.plus(bumpOffset);
            leftPoint = leftPoint.plus(new Translation2d(kBumpWidth, Units.Meters.zero()));
        } else { // Trench
            Translation2d trenchOffset = new Translation2d(
                (!isRightSide ? kBumpWidth.times(2).plus(kTrenchWidth) // If on the left, add everything thats to the right of it
                    : Units.Meters.zero())
                , Units.Meters.zero());

            rightPoint = rightPoint.plus(trenchOffset);
            leftPoint = leftPoint.plus(trenchOffset);
            leftPoint = leftPoint.plus(new Translation2d(kTrenchWidth, Units.Meters.zero()));
        }

        leftPoint = leftPoint.plus(new Translation2d(Units.Meters.zero(), kDepth.times(isDSside ? -1 : 1).times(isBlueSide ? 1 : -1)));
        
        return new Rectangle2d(
            new Translation2d(rightPoint.getMeasureY(), rightPoint.getMeasureX()), // Convert coordinate system
            new Translation2d(leftPoint.getMeasureY(), leftPoint.getMeasureX()));
    }

    public static Translation2d trenchPoint(boolean isBump, boolean isBlueSide, boolean isRightSide) {
        Translation2d rightPoint = kRightBlueTrenchCenterSidePoint;
        Translation2d leftPoint = kRightBlueTrenchCenterSidePoint;

        if (!isBlueSide) {
            isRightSide = !isRightSide;
        }

        Translation2d trenchOffset = new Translation2d(
            (!isRightSide ? kBumpWidth.times(2).plus(kTrenchWidth) // If on the left, add everything thats to the right of it
                : Units.Meters.zero())
            , Units.Meters.zero());

        rightPoint = rightPoint.plus(trenchOffset);
        leftPoint = leftPoint.plus(trenchOffset);
        leftPoint = leftPoint.plus(new Translation2d(kTrenchWidth, Units.Meters.zero()));

        return new Translation2d(rightPoint.getMeasureY(), rightPoint.getMeasureX()).interpolate(new Translation2d(leftPoint.getMeasureY(), leftPoint.getMeasureX()), 0.5);
    }

    public static class ZoneInfo {
        public boolean isBump = false;
        public boolean isBlueSide = false;
        public boolean isDSside = false;
        public boolean isRightSide = false;
        public boolean isWithinOne = false;

        static public ZoneInfo none() {
            return new ZoneInfo();
        }

        public String shortName() {
            if (!isWithinOne) return "None";
            
            return (isBlueSide ? "Blue " : "Red ")
                + (isBump ? "Bump " : "Trench ")
                + (isRightSide ? "Right" : "Left");
        }

        public String fullName() {
            if (!isWithinOne) return "None";

            return shortName() + (isDSside ? " Close" : " Far");
        }
    }

    public static interface Zones {
        Rectangle2d kLeftBlueTrench = constructZone(false, true, false, false);
        Rectangle2d kRightBlueTrench = constructZone(false, true, false, true);
        Rectangle2d kLeftRedTrench = constructZone(false, false, false, false);
        Rectangle2d kRightRedTrench = constructZone(false, false, false, true);
    }

    private static LinkedHashSet<Pair<Rectangle2d, ZoneInfo>> zoneCache = new LinkedHashSet<Pair<Rectangle2d, ZoneInfo>>();
    public static LinkedHashSet<Translation2d> trenchCache = new LinkedHashSet<Translation2d>();

    static {
        for (int i = 0; i < 0x0F; i++) {
            boolean isBump =      (i & 0b0001) != 0;
            boolean isBlueSide =  (i & 0b0010) != 0;
            boolean isDSside =    (i & 0b0100) != 0;
            boolean isRightSide = (i & 0b1000) != 0;

            Rectangle2d rect = constructZone(isBump, isBlueSide, isDSside, isRightSide);
            ZoneInfo zoneInfo = new ZoneInfo();
            zoneInfo.isBump = isBump;
            zoneInfo.isBlueSide = isBlueSide;
            zoneInfo.isDSside = isDSside;
            zoneInfo.isRightSide = isRightSide;

            zoneCache.add(new Pair<Rectangle2d,ZoneConstants.ZoneInfo>(rect, zoneInfo));
            if (isDSside) {
                Translation2d trenchPoint = trenchPoint(isBump, isBlueSide, isRightSide);
                trenchCache.add(trenchPoint);
            }
        }
    }

    public static ZoneInfo checkZone(Translation2d position) {
        ZoneInfo determined = new ZoneInfo();
        zoneCache.forEach(zone -> {
            Rectangle2d rect = zone.getFirst();
            ZoneInfo zoneInfo = zone.getSecond();
            if (rect.contains(position)) {
                determined.isBump = zoneInfo.isBump;
                determined.isBlueSide = zoneInfo.isBlueSide;
                determined.isDSside = zoneInfo.isDSside;
                determined.isRightSide = zoneInfo.isRightSide;
                determined.isWithinOne = true;
            }
        });
        return determined;
    }

    public static Translation2d closestTrench(Translation2d position) {
        Translation2d closestPoint = new Translation2d();
        double closestDistance = 1E9d;
        for (Translation2d trenchPoint : trenchCache) {
            double dist = position.getDistance(trenchPoint);
            if (dist < closestDistance) {
                closestPoint = trenchPoint;
                closestDistance = dist;
            }
        }

        return closestPoint;
    }
}
