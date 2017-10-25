package org.firstinspires.ftc.teamcode.TechNova2017;

import java.util.ArrayDeque;

public class MovingAverage {
    ArrayDeque<Double> queue;
    int size;
    double avg;

    /** Initialize your data structure here. */
    public MovingAverage(int size) {
        this.queue = new ArrayDeque<Double>();
        this.size = size;
    }

    public double next(double val) {
        if(queue.size()<this.size){
            queue.offer(val);
            double sum=0;
            for(double i: queue){
                sum+=i;
            }
            avg = (double)sum/queue.size();

            return avg;
        }else{
            double head = queue.poll();
            double minus = (double)head/this.size;
            queue.offer(val);
            double add = (double)val/this.size;
            avg = avg + add - minus;
            return avg;
        }
    }
}
