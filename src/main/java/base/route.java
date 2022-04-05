package base;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.stream.Collectors;

public class route implements Cloneable {
    public double cost = 0;
    public double Q;
    public double pr = 0;
    public int profit = 0;
    public ArrayList<Double> time = new ArrayList<>();
    public ArrayList<Double> dis_cost = new ArrayList<>();
    public ArrayList<Integer> service_cost = new ArrayList<>();
    public boolean flag = true;
    // first resource: cost (e.g. distance or strict travel time)

    public ArrayList<Integer> path;


    public route() {
        this.path = new ArrayList<Integer>();
        this.cost = 0.0;
    }

    public route(int pathSize) {
        this.path = new ArrayList<>(pathSize);
        this.cost = 0.0;
    }

    // method for deep cloning
    @SuppressWarnings("unchecked")
    public route clone() throws CloneNotSupportedException {
        route route = (route) super.clone();
        route.path = (ArrayList<Integer>) path.clone();
        return route;
    }

    public void removeCity(int city) {
        this.path.remove(Integer.valueOf(city));

    }

    public void addcity(int f_city, int city) {
        int index = this.path.indexOf(f_city);
        this.path.add(index + 1, city);
    }

    public void addcity(int city) {
        this.path.add(city);
    }

    public void setcost(double c) {
        this.cost = c;
    }

    public double getcost() {
        return this.cost;
    }

    public void setQ(double a) {
        this.Q = a;
    }

    public double getQ() {
        return this.Q;
    }

    public ArrayList<Integer> getpath() {
        return this.path;
    }

    public void switchpath() {
        Integer swap;
        int nb = path.size() / 2;
        for (int i = 0; i < nb; i++) {
            swap = path.get(i);
            path.set(i, path.get(path.size() - 1 - i));
            path.set(path.size() - 1 - i, swap);
        }
    }

    public void setPr(double pr) {
        this.pr = pr;
    }

    public boolean checkTW(paramsVRP instance) {
        //
        //ai aj
        //cost
        profit = 0;
        time = new ArrayList<>();
        double pre_a = 0.0;
        time.add(pre_a);
        int pre = 0;
        for (int i = 1; i < this.path.size(); i++) {
            double dis = instance.distBase[this.path.get(pre)][this.path.get(i)];
            this.dis_cost.add(dis);
            this.service_cost.add(instance.s[this.path.get(pre)]);
            double new_a = pre_a + dis + instance.s[this.path.get(pre)];
            if (new_a < instance.a[this.path.get(i)]) {
                new_a = instance.a[this.path.get(i)];
            }
            if (new_a > instance.b[this.path.get(i)]) {
                flag = false;
            }
            pre = i;
            time.add(new_a);
            pre_a = new_a;
            //b[i]
            this.profit += instance.profit[this.path.get(i)];
        }

        return true;
    }
    public void checkDistance(paramsVRP instance) {
        //ai aj
        checkTW(instance);
        double totalDis = this.dis_cost.stream()
                .collect(Collectors.summingDouble(Double::doubleValue));
        //System.out.println("route_dis = "+totalDis+",dis = "+this.dis_cost.toString());
    }


    public boolean checkCost() {
        return true;
    }

    public void write2log(BufferedWriter bw1) throws IOException {
        bw1.write("\n Path: " + this.path.toString() + " profit: " + this.profit + " ,cost: " + this.cost + ",pr: " + this.pr);
        bw1.write("\n Distance = " + this.dis_cost.toString());
        bw1.write("\n Service = " + this.service_cost.toString());
        bw1.write("\n Arrive_Time = " + this.time.toString() + " checkTime = " + this.flag);
        bw1.flush();
    }
    //生成一条路径 就 记录 node_seq, cost, Q pr 每个点到达的时间 路径cost b[i]的概率
    public void write2pw(PrintWriter bw1) throws IOException {
        bw1.write("\n Path: " + this.path.toString() + " profit: " + this.profit + " ,cost: " + this.cost + ",pr: " + this.pr);
        bw1.write("\n Distance = " + this.dis_cost.toString());
        bw1.write("\n Service = " + this.service_cost.toString());
        bw1.write("\n Arrive_Time = " + this.time.toString() + " checkTime = " + this.flag);
        bw1.flush();
    }


}