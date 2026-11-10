package frc.robot.util;

import java.lang.reflect.Method;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.FieldRegions;

public class RegionUtil {
    private RegionUtil() {}

    // *Imports RegionBool zone as Zone class

    public static Zone fromRegionBool(String zoneName) {
        try {
            Class<?> clazz = FieldRegions.class;
            String functionName = getFunctionName(zoneName);
            Method containsMethod = clazz.getMethod(functionName, Translation2d.class);

            return new Zone() {
                @Override
                public boolean contains(Translation2d point) {
                    try {
                        return (boolean) containsMethod.invoke(null, point);
                    } catch (Exception e) {
                        throw new RuntimeException("Failed to invoke region check: " + functionName, e);
                    }
                }

                @Override
                public Pose2d[] getPoints() {
                    return null;
                }
            };
        } catch (Exception e) {
            System.out.println("Trenches failed to load lil bro");
            throw new IllegalArgumentException("No matching region method found for: " + zoneName, e);
        }
    }

    //Formats the function name how FieldRegions does it. Very very similar to PascalCase
    private static String getFunctionName(String input) {
        if (input == null || input.isEmpty()) {
            return input;
        }

        String[] words = input.trim().split("\\s+");
        StringBuilder result = new StringBuilder();

        for (String word : words) {
            if (!word.isEmpty()) {
                result.append(Character.toUpperCase(word.charAt(0)));
                if (word.length() > 1) {
                    result.append(word.substring(1));
                }
            }
        }

        return "isIn" + result.toString();
    }
}
