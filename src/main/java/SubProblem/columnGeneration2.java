package SubProblem;

import java.io.BufferedWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import Regression.LRRegression;
import base.IloNumVarArray;
import base.paramsVRP;
import base.route;
import ilog.concert.*;
import ilog.cplex.*;
import org.apache.log4j.Logger;
import util.Stopwatch;
import weka.classifiers.functions.LinearRegression;


public class columnGeneration2 {
    private static Logger logger = Logger.getLogger(columnGeneration2.class);
    public LRRegression LR = new LRRegression();
    public LinearRegression lr = new LinearRegression();
    public int lrFunction = 0;

    public double computeColGen(paramsVRP userParam, ArrayList<route> routes,
                                BufferedWriter bw1, BufferedWriter bw2, BufferedWriter bw3, BufferedWriter bw4,
                                boolean isOnePass, boolean isMaxcnt,boolean isPrediction)
            throws Exception {
        int i, j, prevcity, city;
        double cost, obj = 0;
        double[] pi;
        boolean oncemore;

        ArrayList<double[]> dualPrice = new ArrayList<>();
        ArrayList<Double> Qvalue = new ArrayList<>();
        double curQ = 0.0;
        double routeQ = 0.0;


        //try {

        // ---------------------------------------------------------
        // construct the model for the Restricted Master Problem
        // ---------------------------------------------------------
        Stopwatch timer1 = new Stopwatch();
        IloCplex cplex = new IloCplex();

        IloNumVar[] k = new IloNumVar[userParam.nbclients + 2];
        for (i = 0; i < userParam.nbclients + 2; i++)
            k[i] = cplex.numVar(-1, -1, "k" + i);
        IloObjective objfunc = cplex.addMinimize(cplex.scalProd(k, userParam.profit));

        //cplex.column(objfunc,-100);
        // for each constraint, right member + vi =1
        IloRange[] lpmatrix = new IloRange[userParam.nbclients];
        for (i = 0; i < userParam.nbclients; i++)
            lpmatrix[i] = cplex.addRange(1.0, 1.0);//Double.MAX_VALUE

        // Declaration of the variables
        IloNumVarArray y = new IloNumVarArray(); // y_p to define whether a path p is used
        IloRange tmp = cplex.addRange(0, userParam.vehicle_number);
        IloNumVar[] z = new IloNumVar[userParam.nbclients];

        // Populate the lp matrix and the objective function
        // first with the routes provided by the argument 'routes' of the function
        // (in the context of the Branch and Bound, it would be a pity to start
        // again the CG from scratch at each node of the BB!)
        // (we should reuse parts of the previous solution(s))
        for (route r : routes) {
            int v;
            cost = 0.0;
            prevcity = 0;
            for (i = 1; i < r.getpath().size(); i++) {
                city = r.getpath().get(i);
                cost += userParam.dist[prevcity][city];
                prevcity = city;
            }
            r.setcost(cost);
            //Objective
            IloColumn column = cplex.column(objfunc, userParam.costWeight * r.getcost());
            column = column.and(cplex.column(tmp, 1));
            // obj coefficient
            for (i = 1; i < r.getpath().size() - 1; i++) {
                v = r.getpath().get(i) - 1;
                column = column.and(cplex.column(lpmatrix[v], 1.0));
                // coefficient of y_i => 0 for the other y_p
            }
            y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
            // creation of the variable y_i
        }

        // complete the lp with basic route to ensure feasibility TODO
        if (routes.size() < userParam.nbclients) { // a priori true only the first time
            for (i = 0; i < userParam.nbclients; i++) {
                cost = userParam.dist[0][i + 1]
                        + userParam.dist[i + 1][userParam.nbclients + 1];
                IloColumn column = cplex.column(objfunc, userParam.costWeight * cost); // obj coefficient
                column = column.and(cplex.column(lpmatrix[i], 1.0)); // coefficient of
                column = column.and(cplex.column(tmp, 1));
                y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
                route newroute = new route();
                newroute.addcity(0);
                newroute.addcity(i + 1);
                newroute.addcity(userParam.nbclients + 1);
                newroute.setcost(cost);
                routes.add(newroute);
            }
        }

        for (i = 1; i < userParam.nbclients + 1; i++) {
            //profit 仅在此处出现
            IloColumn column = cplex.column(objfunc, userParam.profitWeight * userParam.profit[i]);
            //cplex.column(objfunc, -1 * userParam.profitWeight * userParam.profit[i]);
            //logger.info("column" + objfunc.toString());
            column = column.and(cplex.column(lpmatrix[i - 1], 1.0));
            //todo 限制z的取值
            z[i - 1] = cplex.numVar(column, 0, 1, "z" + (i - 1));//是否访问该顾客点
        }

        cplex.exportModel("./dataset/cplex_model/MP_Model.lp");

        // CPLEX params
        cplex.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Primal);
        cplex.setOut(null);
        double time1 = timer1.elapsedTime();
        logger.info("Rebuild RMP_model cost " + time1);

        // cplex.setParam(IloCplex.DoubleParam.TiLim,30); // max number of
        // seconds: 2h=7200 24h=86400

        // ---------------------------------------------------------
        // column generation process
        // ---------------------------------------------------------
        DecimalFormat df = new DecimalFormat("#0000.00");
        oncemore = true;
        //oncemore = false;
        double[] prevobj = new double[100];
        int nbroute;
        int previ = 0;
        //System.err.println(!(isMaxcnt ^ previ < userParam.maxcnt));

        while (oncemore) {
            if (isMaxcnt) {
                if (previ >= userParam.maxcnt) {
                    System.err.println(previ + ",break");
                    break;
                }
            }

            oncemore = false;
            // ---------------------------------------s------------------
            // solve the current RMP
            // ---------------------------------------------------------
            Stopwatch timer3 = new Stopwatch();


            if (!cplex.solve()) {
                System.out.println("CG: relaxation infeasible!");
                return 1E10;
            }
            prevobj[(previ) % 100] = cplex.getObjValue();
            double time3 = timer3.elapsedTime();
            System.out.println("Solve RMP_model cost " + time3 + " obj " + cplex.getObjValue());
            // store the 30 last obj values to check stability afterwards

            // System.out.println(cplex.getStatus());
            cplex.exportModel("./dataset/cplex_model/model" + Integer.toString(previ) + ".lp");
            previ++;
            // ---------------------------------------------------------
            // solve the subproblem to find new columns (if any)
            // ---------------------------------------------------------
            // first define the new costs for the subproblem objective function
            pi = cplex.getDuals(lpmatrix);
            double last_dual = cplex.getDual(tmp);
            bw1.write("\n userParam.dual_cost");
            bw1.flush();

            for (j = 0; j < userParam.nbclients + 2; j++) {
                bw1.write("\n i,j=" + 0 + "," + j + "," + userParam.cost[0][j]);
                bw1.flush();
            }

            for (i = 1; i < userParam.nbclients + 1; i++) {
                for (j = 0; j < userParam.nbclients + 2; j++) {
                    userParam.cost[i][j] = userParam.dist[i][j] - pi[i - 1];
                    bw1.write("\n i,j=" + i + "," + j + "," + userParam.cost[i][j]);
                    bw1.flush();
                }
            }

            //new prediction todo
            if (previ >= 2) {
                double[] testList = new double[pi.length - 1];
                for (int p = 0; p < pi.length - 1; p++) {
                    testList[p] = pi[p];
                }
                //testList[pi.length] = last_dual;
                curQ = LR.countLinerRegression(testList, lr);
                bw4.write("\n CG iter = " + previ + " testPrice = " + Arrays.toString(testList) +
                        " \n Predict Qvalue = " + curQ);
                bw4.flush();

            }
            if (!isPrediction) {
                curQ = 100;
            }
            // start dynamic programming
            Stopwatch timer2 = new Stopwatch();

            //SPPRC
            SPPRC sp = new SPPRC();
            ArrayList<route> routesSPPRC = new ArrayList<route>();

            nbroute = userParam.nbclients; // arbitrarily limit to the 5 first
            // shortest paths with negative cost
            nbroute = 3;

            bw2.write("\n CG ITER = " + previ);
            sp.shortestPath(userParam, routesSPPRC, nbroute, bw2, bw3, curQ);
            lrFunction += sp.changeCnt;
            routeQ = sp.routeQ;
            bw2.flush();
            sp = null;
            double time2 = timer2.elapsedTime();
            System.out.println("SubProblem SPPRC cost " + time2 + " SubRoute = " +
                    routesSPPRC.size() + " curQ= " + curQ + " routeQ = " + routeQ);
            // parameter here
            if (routesSPPRC.size() > 0) {
                for (route r : routesSPPRC) {

//             if (userParam.debug) {
//             logger.info(" "+r.getcost());
//             }
                    ArrayList<Integer> rout = r.getpath();
                    prevcity = rout.get(1);
                    cost = userParam.dist[0][prevcity];
                    IloColumn column = cplex.column(lpmatrix[rout.get(1) - 1], 1.0);
                    column = column.and(cplex.column(tmp, 1));
                    for (i = 2; i < rout.size() - 1; i++) {
                        city = rout.get(i);
                        cost += userParam.dist[prevcity][city];
                        prevcity = city;
                        column = column.and(cplex.column(lpmatrix[rout.get(i) - 1], 1.0));
                        // coefficient of y_i in (3.23) => 0 for the other y_p
                    }
                    cost += userParam.dist[prevcity][userParam.nbclients + 1];
                    column = column.and(cplex.column(objfunc, cost * userParam.costWeight));
                    y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE,
                            "P" + routes.size())); // creation of the variable y_i
                    r.setcost(cost);
                    routes.add(r);

                    oncemore = true;


                }
                logger.info("\nCG Iter " + previ + " Current cost: "
                        + df.format(prevobj[(previ - 1) % 100]) + ",routes.size=" + routes.size());
            }
            //get dual/route_cost
            double[] curDual = new double[pi.length - 1];
            for (int p = 0; p < pi.length - 1; p++) {
                curDual[p] = pi[p];
            }
            //curDual[pi.length] = last_dual;
            dualPrice.add(curDual);
            Qvalue.add(routeQ);
            //update lr
            lr = LR.doLinearRegression(dualPrice, Qvalue);
            //
            bw4.write("\n CG iter = " + previ + " dualPrice = ");
            //dualPrice.stream().forEach(e->bw4.write(Arrays.toString(e).toString()));
            for (double[] e : dualPrice) {
                bw4.write(Arrays.toString(e).toString());
            }
            bw4.write("\n Qvalue = " + Qvalue.toString() + " lrFunction  = " + lrFunction);
            bw4.flush();
            //if (previ % 50 == 0)
            routesSPPRC = null;
            //System.err.println("PREVI = " + previ);
        }

        // should recompute the obj using the distBase
        // matrix instead of the dist matrix
        if (isOnePass) {
            for (int i1 = 0; i1 < y.getSize(); i1++) {
                cplex.add(cplex.conversion(y.getElement(i1), IloNumVarType.Int));
            }
            cplex.exportModel("./dataset/cplex_model/finalIntIP.lp");
            if (!cplex.solve()) {
                System.out.println("OnePass Failed ");
            } else {
                for (i = 0; i < y.getSize(); i++)
                    routes.get(i).setQ(cplex.getValue(y.getElement(i)));
                obj = cplex.getObjValue();
                System.out.println("OnePass Obj = " + obj);
            }
        } else {
            for (i = 0; i < y.getSize(); i++)
                routes.get(i).setQ(cplex.getValue(y.getElement(i)));
            obj = cplex.getObjValue(); // mmmmhhh: to check. To be entirely safe, we
            System.err.println("OBJ = " + obj);
        }

        cplex.end();
        return obj;

        //} catch (IloException e) {
        //logger.error("Concert exception caught " + e + "caught");
        //}
        //return 1E10;
    }
}
