package org.firstinspires.ftc.teamcode.pedroPathing;
import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {

    //this class was written by AI
    private Queue<Double> values;
    private final int capacity;
    private double sum;

    public MovingAverage(int capacity) {
        this.capacity = capacity;
        this.values = new LinkedList<>();
        this.sum = 0;
    }

    public void addValue(double newValue) {
        // If the queue is at capacity, remove the oldest value
        if (values.size() == capacity) {
            sum -= values.poll(); // Remove oldest and subtract from sum
        }
        // Add the new value
        values.offer(newValue);
        sum += newValue;
    }

    public double getAverage() {
        if (values.isEmpty()) {
            return 0.0; // Avoid division by zero
        }
        return sum / values.size();
    }

    public static void main(String[] args) {
        MovingAverage ma = new MovingAverage(5); // Average of last 5 values

        ma.addValue(10);
        System.out.println("Current average: " + ma.getAverage()); // 10.0

        ma.addValue(20);
        System.out.println("Current average: " + ma.getAverage()); // 15.0

        ma.addValue(30);
        System.out.println("Current average: " + ma.getAverage()); // 20.0

        ma.addValue(40);
        System.out.println("Current average: " + ma.getAverage()); // 25.0

        ma.addValue(50);
        System.out.println("Current average: " + ma.getAverage()); // 30.0

        ma.addValue(60); // 10 is removed, 60 is added
        System.out.println("Current average: " + ma.getAverage()); // (20+30+40+50+60)/5 = 40.0
    }
}