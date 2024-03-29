package frc.robot;

import java.util.ArrayList;

public class WeightedAverageController {
    public ArrayList<Double> memory = new ArrayList<>();
    public int memoryLength;

    public WeightedAverageController(int memory_length) {
        this.memoryLength = memory_length;
    }
    
    public double calculate(double latest) {
        if(memory.size() > memoryLength) {
            memory.remove(memoryLength);
        }

        memory.add(0, latest);
        
        double totalWeight = 0;
        double weightedSum = 0;

        for (int i = 0; i < memory.size(); i++) {
            double weight = Math.pow(0.5, i);
            weightedSum += memory.get(i) * weight;
            totalWeight += weight;
        }

        return weightedSum / totalWeight;
    }

    public void forget() {
        memory.clear();
    }
}
