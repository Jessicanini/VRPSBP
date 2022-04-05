package MasterProblem;

import SubProblem.columnGeneration2;
import base.paramsVRP;
import base.route;
import org.apache.log4j.Logger;
import org.junit.Test;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class VRPSSSolver {
    private static Logger logger = Logger.getLogger(VRPSSSolver.class);

    @Test
    public void test() throws IOException {

        logger.info("Start ");
        double time1 = System.currentTimeMillis();
        int maxcnt = 150;
        boolean isOnePass = false;
        boolean isMaxcnt = false;
        boolean isPrediction = true;//false 不预测
        int nodenum = 25;
        double costWeight = 1.0;
        double profitWeight = 1.0;
        String prefix = "R108_25";
        paramsVRP instance = new paramsVRP(nodenum, maxcnt);
        instance.initParams("./dataset/inputdata/" + prefix + ".txt", costWeight, profitWeight);
        instance.setServiceTime(8, 10, 12);
        //R类	10	[8,12]
        //RC类	10	[8,12]
        //C类	90	[70,110]
        ArrayList<route> initRoutes = new ArrayList<route>();
        ArrayList<route> bestRoutes = new ArrayList<route>();
        String file1 = "./dataset/" + prefix + "dual_price.txt";
        FileWriter fw1 = new FileWriter(makefile(file1), false);
        BufferedWriter bw1 = new BufferedWriter(fw1);
        String file2 = "./dataset/" + prefix + "route_log.txt";
        FileWriter fw2 = new FileWriter(makefile(file2), false);
        BufferedWriter bw2 = new BufferedWriter(fw2);
        String file3 = "./dataset/" + prefix + "Pr_log.txt ";
        FileWriter fw3 = new FileWriter(makefile(file3), false);
        BufferedWriter bw3 = new BufferedWriter(fw3);

        String file4 = "./dataset/" + prefix + "LR_log.txt ";
        FileWriter fw4 = new FileWriter(makefile(file4), false);
        BufferedWriter bw4 = new BufferedWriter(fw4);

        FileWriter fw = new FileWriter("./dataset/" + prefix + "result.txt");
        BufferedWriter bw = new BufferedWriter(fw);
        PrintWriter pw = new PrintWriter(bw);

        BranchAndBound bp = new BranchAndBound(bw1, bw2, bw3, bw4);
        bp.BBnode(instance, initRoutes, null, bestRoutes, 0, isOnePass, isMaxcnt,isPrediction);
        double time2 = System.currentTimeMillis();
        //columnGeneration2 cg = new columnGeneration2();
        //cg.computeColGen(instance, initRoutes, bw1, bw2, bw3);

        //initRoutes
        pw.println("Initial Routes: ");
        for (route r : initRoutes) {
            //logger.info(r.path.toString() + "," + r.getcost() + "," + r.getQ());
            pw.println(r.path.toString() + "," + r.getcost() + "," + r.getQ());
            pw.flush();
        }

        //bp.BBnode(instance, initRoutes, null, bestRoutes, 0);
        double optCost = 0;
        double optProfit = 0;
        pw.println("Solution >>>");
        pw.flush();
        System.out.println("Solution >>> ");
        System.out.println(prefix + " , " + maxcnt);
        ArrayList<Integer> allC = new ArrayList<>();
        for (int i = 0; i < instance.nbclients + 1; i++) {
            allC.add(i);
        }
        ArrayList<Integer> tmpC = new ArrayList<>();
        for (int i = 0; i < bestRoutes.size(); ++i) {
            System.out.print(bestRoutes.get(i).path + ",");
            bestRoutes.get(i).checkDistance(instance);
            //System.out.println();
            optCost += bestRoutes.get(i).cost;
            bestRoutes.get(i).checkTW(instance);
            tmpC.addAll(bestRoutes.get(i).path);
            optProfit += bestRoutes.get(i).profit;
            bestRoutes.get(i).write2pw(pw);
            pw.flush();
        }
        System.out.println();
        List<Integer> unvisitedC = allC.stream().filter(c -> !tmpC.contains(c)).collect(Collectors.toList());
        pw.println("\n 该解的单目标函数值为 " + (optCost - optProfit) + " 收集到的利润 = " + optProfit + " 路线距离 ="
                + optCost + " 未访问的顾客: " + unvisitedC);
        System.out.println(optProfit + " , " + optCost + " , " + unvisitedC);
        System.out.println("该解的单目标函数值为 " + (optCost - optProfit) + " 收集到的利润 = " + optProfit + " 路线距离 ="
                + optCost + " 未访问的顾客: " + unvisitedC);
        System.out.println("该解的耗时为 " + (time2 - time1) / 1000.0 + " seconds ");


        pw.flush();
        logger.info("Best Cost = " + optCost);
    }

    public File makefile(String fp) {
        File file = null;
        try {
            file = new File(fp);
            if (file.exists()) {
                file.delete();
            }//delete file!!!
            if (!file.getParentFile().exists()) {
                boolean mkdir = file.getParentFile().mkdirs();
                if (!mkdir) {
                    throw new RuntimeException("Fail");
                }
            }
            if (!file.exists()) {
                file.createNewFile();
            }
        } catch (IOException e) {
        }
        return file;
    }

}


