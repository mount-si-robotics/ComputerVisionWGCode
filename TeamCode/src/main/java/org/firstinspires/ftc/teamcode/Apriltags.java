package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

public class Apriltags {
    public static void goToTag(LLResult tagResult, int tag) {
        if (tagResult == null || !tagResult.isValid()) return;
        List<LLResultTypes.FiducialResult> results = tagResult.getFiducialResults();
        for (LLResultTypes.FiducialResult result : results) {
            int thisTag = result.getFiducialId();
            double x = result.getTargetXPixels();
            double angle = result.getTargetXDegrees();
            double area = result.getTargetArea();
        }
    }
}
