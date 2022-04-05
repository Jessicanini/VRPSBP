package SubProblem;

import base.paramsVRP;
import base.route;
import org.apache.log4j.Logger;

import java.io.BufferedWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.TreeSet;

import static java.lang.Math.max;
import static java.lang.Math.min;

// shortest path with resource constraints
// inspired by Irnish and Desaulniers, "SHORTEST PATH PROBLEMS WITH RESOURCE CONSTRAINTS"
// for educational demonstration only - (nearly) no code optimization
//
// four main lists will be used:
// labels: array (ArraList) => one dimensional unbounded vector
//		 list of all labels created along the feasible paths (i.e. paths satisfying the resource constraints)
//
// U: sorted list (TreeSet) => one dimensional unbounded vector
//		sorted list containing the indices of the unprocessed labels (paths that can be extended to obtain a longer feasible path)
//
// P: sorted list (TreeSet) => one dimensional unbounded vector
//		sorted list containing the indices of the processed labels ending at the depot with a negative cost
//
// city2labels: matrix (array of ArrayList) => nbclients x unbounded
//		for each city, the list of (indices of the) labels attached to this city/vertex
//		before processing a label at vertex i, we compare pairwise all labels at the same vertex to remove the dominated ones

public class SPPRC {
    paramsVRP userParam;
    ArrayList<label> labels;
    private static Logger logger = Logger.getLogger(SPPRC.class);
    public double routeQ = -1000.0;
    public double maxRouteQ = -10000.0;
    public double minRouteQ = 0.0;
    public int changeCnt = 0;


    class label {
        // we use a labelling algorith.
        // labels are attached to each vertex to specify the state of the resources when we follow a corresponding feasible path ending at this vertex
        public int city;                // current vertex
        public int indexPrevLabel;    // previous label in the same path (i.e. previous vertex in the same path with the state of the resources)

        public double cost;                // reduced_cost
        public double cost_noproba;
        public double tworst;
        public double demand;                // third resource: demand,i.e. total quantity delivered to the clients encountered on this path

        public double Tmax;
        public double Trec;

        public double ttime;                // second resource: travel time along the path (including wait time and service time)

        public boolean dominated;            // is this label dominated by another one? i.e. if dominated, forget this path.
        public boolean[] vertexVisited;
        public double[] Mt_;
        public double Pr;


        label(int a1, int a2, double a3, double a3_1, double a4_1, double a4_2, double a5, double a6,
              boolean a7, boolean[] a8, double[] a9, double a10, double a11) {
            city = a1;
            indexPrevLabel = a2;

            cost = a3;
            cost_noproba = a3_1;

            tworst = a4_1;
            demand = a4_2;

            Tmax = a5;
            Trec = a6;

            dominated = a7;
            vertexVisited = a8;
            Mt_ = a9;
            Pr = a10;
            ttime = a11;
        }
    }

    class MyLabelComparator implements Comparator<Integer> {
        // the U treeSet is an ordered list
        // to maintain the order, we need to define a comparator: cost is the main criterium
        public int compare(Integer a, Integer b) {
            label A = labels.get(a);
            label B = labels.get(b);

            // Be careful!  When the comparator returns 0, it means that the two labels are considered EXACTLY the same ones!
            // This comparator is not only used to sort the lists!  When adding to the list, a value of 0 => not added!!!!!
            if (A.cost - B.cost < -1e-7)//TODO
                return -1;
            else if (A.cost - B.cost > 1e-7)
                return 1;
            else {
                if (A.city == B.city) {
                    if (A.demand - B.demand < -1e-7)
                        return -1;
                    else if (A.demand - B.demand > 1e-7)
                        return 1;
                    else {
                        int i = 0;
                        while (i < userParam.nbclients + 2) {
                            if (A.vertexVisited[i] != B.vertexVisited[i]) {
                                if (A.vertexVisited[i])
                                    return -1;
                                else
                                    return 1;
                            }
                            i++;
                        }
                        return 0;
                    }
                } else if (A.city > B.city)
                    return 1;
                else
                    return -1;
            }
        }
    }


    public void shortestPath(paramsVRP userParamArg, ArrayList<route> routes, int nbroute, BufferedWriter bw1,
                             BufferedWriter bw2, double curQ) throws IOException {
        label current;
        int idx, nbsol, maxsol;
        double d, d2;
        int[] checkDom;
        double tt, tt2;
        Integer currentidx;

        this.userParam = userParamArg;
        // unprocessed labels list => ordered TreeSet List (?optimal:  need to be sorted like this?)
        TreeSet<Integer> U = new TreeSet<Integer>(new MyLabelComparator());   // unprocessed labels list

        // processed labels list => ordered TreeSet List
        TreeSet<Integer> P = new TreeSet<Integer>(new MyLabelComparator());   // unprocessed labels list

        // array of labels
        labels = new ArrayList<label>(2 * userParam.nbclients); // initial size at least larger than nb clients

        boolean[] cust = new boolean[userParam.nbclients + 2];//
        cust[0] = true;
        for (int i = 1; i < userParam.nbclients + 2; i++) {
            cust[i] = false;
        }

        //initial time-fea pro
        double[] Mt0_ = new double[userParam.b[0] * 2];
        for (int i1 = 0; i1 < Mt0_.length; i1++) {
            Mt0_[i1] = 1;
        }
        double p0 = Mt0_[userParam.b[0]];

        //city pre cost demand Tmax Trec
        labels.add(new label(0, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, cust, Mt0_, p0, 0));    // first label: start from depot (client 0)
        U.add(0);

        // for each city, an array with the index of the corresponding labels (for dominance)
        checkDom = new int[userParam.nbclients + 2];
        ArrayList<Integer>[] city2labels = new ArrayList[userParam.nbclients + 2];
        for (int i = 0; i < userParam.nbclients + 2; i++) {
            city2labels[i] = new ArrayList<Integer>();
            checkDom[i] = 0;  // index of the first label in city2labels that needs to be checked for dominance (last labels added)
        }
        city2labels[0].add(0);

        nbsol = 0;
        maxsol = 2 * nbroute;
        while ((U.size() > 0) && (nbsol < maxsol)) {
            // second term if we want to limit to the first solutions encountered to speed up the SPPRC (perhaps not the BP)
            // remark: we'll keep only nbroute, but we compute 2xnbroute!  It makes a huge difference=>we'll keep the most negative ones
            // this is something to analyze further!  how many solutions to keep and which ones?
            // process one label => get the index AND remove it from U
            currentidx = U.pollFirst();
            current = labels.get(currentidx);
            //logger.info("Current= "+current.city);

            // check for dominance
            int l1, l2;
            boolean pathdom;
            boolean prdom;
            label la1, la2;
            ArrayList<Integer> cleaning = new ArrayList<Integer>();
            for (int i = checkDom[current.city]; i < city2labels[current.city].size(); i++) {  // check for dominance between the labels added since the last time we came here with this city and all the other ones
                for (int j = 0; j < i; j++) {
                    l1 = city2labels[current.city].get(i);
                    l2 = city2labels[current.city].get(j);
                    la1 = labels.get(l1);
                    la2 = labels.get(l2);//dominate
                    if (!(la1.dominated || la2.dominated)) {  // could happen since we clean 'city2labels' thanks to 'cleaning' only after the double loop
                        pathdom = true;
                        prdom = true;
                        for (int k = 1; pathdom && (k < userParam.nbclients + 2); k++)
                            pathdom = (!la1.vertexVisited[k] || la2.vertexVisited[k]);
                        //可行性的概率 todo
                        for (int k = userParam.a[current.city]; prdom && (k < userParam.b[current.city]); k++)
                            prdom = prdom && greater2eq(la1.Mt_[k], la2.Mt_[k]);

                        if (pathdom && prdom && (la1.cost_noproba <= la2.cost_noproba) && (la1.demand <= la2.demand)) {
                            labels.get(l2).dominated = true;
                            U.remove((Integer) l2);
                            cleaning.add(l2);
                            pathdom = false;
                        }
                        pathdom = true;
                        prdom = true;
                        for (int k = 1; pathdom && (k < userParam.nbclients + 2); k++)
                            pathdom = (!la2.vertexVisited[k] || la1.vertexVisited[k]);
                        //如果一样 怎么处理 >=才算占优 所以相等的话也是
                        for (int k = userParam.a[current.city]; prdom && (k < userParam.b[current.city]); k++)
                            prdom = prdom && greater2eq(la2.Mt_[k], la1.Mt_[k]);

                        if (pathdom && prdom && (la2.cost_noproba <= la1.cost_noproba) && (la2.demand <= la1.demand)) {
                            labels.get(l1).dominated = true;
                            U.remove(l1);
                            cleaning.add(l1);
                            j = city2labels[current.city].size();
                        }
                    }
                }
            }

            for (Integer c : cleaning)
                city2labels[current.city].remove((Integer) c);   // a little bit confusing but ok since c is an Integer and not an int!
            cleaning = null;
            checkDom[current.city] = city2labels[current.city].size();  // update checkDom: all labels currently in city2labels were checked for dom.
            //logger.info("Current= " + current.city + " dominated= " + current.dominated);

            // expand REF
            if (!current.dominated) {
                if (current.city == userParam.nbclients + 1) { // shortest path candidate to the depot!
                    //logger.info("Label " + current.city + "," + current.cost_noproba + "," + current.cost + "," + current.ttime + "," + current.dominated);
                    if (current.cost_noproba < -1e-7) {
                        // SP candidate for the column generation
                        // max minnRouteQ
                        P.add(currentidx);
                        nbsol = 0;
                        for (Integer labi : P) {
                            label s = labels.get(labi);
                            if (!s.dominated)
                                nbsol++;
                            maxRouteQ = max(maxRouteQ, s.cost_noproba);
                            minRouteQ = min(minRouteQ, s.cost_noproba);
                        }
                        //maxRouteQ

                    }
                } else {  // if not the depot, we can consider extensions of the path
                    for (int i = 0; i < userParam.nbclients + 2; i++) {
                        if ((!current.vertexVisited[i]) && (userParam.dist[current.city][i] < userParam.verybig - 1e-6)) {  // don't go back to a vertex already visited or along a forbidden edge

                            // ttime TODO probability
                            tt = (float) (current.ttime + userParam.ttime[current.city][i] + userParam.s[current.city]);
                            if (tt < userParam.a[i])
                                tt = userParam.a[i];

                            double[] Mti_ = current.Mt_;//ms[i][k]
                            double[] Mtj_ = new double[userParam.b[0] * 2 + 1];//TODO
                            for (int z = 0; z < userParam.a[i]; z++) {
                                Mtj_[z] = 0;
                            }
                            for (int z = userParam.a[i]; z <= userParam.b[i]; z++) {
                                //logger.info(current.city+","+z+","+userParam.mik[current.city].toString());
                                Mtj_[z] = 0;
                                for (Integer k : userParam.mik[current.city]) {
                                    int idx1 = (int) (z - k - userParam.ttime[current.city][i]);
                                    if (idx1 < 0 /*|| idx1>=2*userParam.b[current.city]*/) {
                                        Mtj_[z] += 0;
                                    } else {
                                        Mtj_[z] += userParam.ms[current.city][k] * Mti_[idx1];
                                    }
                                }
                            }//ai 要写开来么？
                            for (int z = userParam.b[i] + 1; z <= 2 * userParam.b[0]; z++) {
                                Mtj_[z] = Mtj_[userParam.b[i]];
                            }

                            // demand
                            d = current.demand + userParam.d[i];

                            //reduced_cost
                            //pi= Mti_[userParam.b[current.city]] pj Mtj_[userParam.b[j]]
                            double P_i = current.Pr;
                            double P_j = Mtj_[userParam.b[i]];
                            //有概率错过时间窗的cost
                            double cost_j = current.cost + userParam.cost[current.city][i] + userParam.penalty_value * (P_i - P_j);
                            double cost_noprobaj = current.cost_noproba + userParam.cost[current.city][i];

                            double tworst_j = Math.max(userParam.a[i], current.tworst + userParam.ttime[current.city][i] + userParam.smax[current.city]);
                            double Tmax_j = min(Math.max(current.Tmax + userParam.smax[current.city] + userParam.ttime[current.city][i], userParam.a[i]), userParam.b[i]);
                            double Trec_j = 0;
                            //logger.info("cur_i=" + i + " ,pr_j= " + P_j + " d:" + d + " t:" + tt);
                            // is route feasible? a 概率 TODO
                            // logger.info("cur_i= "+current.city+" j:"+i+" pr:"+P_j);
                            if (d <= userParam.capacity && (P_j >= userParam.feasible_pro)) {
                                idx = labels.size();
                                boolean[] newcust = new boolean[userParam.nbclients + 2];
                                System.arraycopy(current.vertexVisited, 0, newcust, 0, userParam.nbclients + 2);
                                newcust[i] = true;
                                //speedup: third technique - Feillet 2004 as mentioned in Laporte's paper
                                for (int j = 1; j <= userParam.nbclients; j++)
                                    if (!newcust[j]) {
                                        tt2 = tt + userParam.ttime[i][j] + userParam.smax[i];//todo
                                        d2 = d + userParam.d[j];
                                        if ((tt2 > userParam.b[j]) || (d2 > userParam.capacity))
                                            newcust[j] = true;  // useless to visit this client
                                    }

                                labels.add(new label(i, currentidx, cost_j, cost_noprobaj, tworst_j, d, Tmax_j, Trec_j, false, newcust, Mtj_, P_j, tt));    // first label: start from depot (client 0)
                                if (!U.add((Integer) idx)) {
                                    // only happens if there exists already a label at this vertex with the same cost, time and demand and visiting the same cities before
                                    // It can happen with some paths where the order of the cities is permuted
                                    labels.get(idx).dominated = true; // => we can forget this label and keep only the other one
                                } else
                                    city2labels[i].add(idx);
                                //logger.info("EXF " + current.city + " to " + i);

                            }
                        }
                    }
                }
            }
        }
        // clean
        checkDom = null;

        bw1.write("\n BEFORE  " + maxRouteQ + " , " + minRouteQ + " , prediction = " + curQ);
        //定义上下界 不至于无解
        //min - max -100 -10
        if (curQ < minRouteQ) {
            curQ = 100.0;
            changeCnt += 1;
        } else {
            if (curQ >= minRouteQ && curQ <= maxRouteQ) {
                //pass
            } else {
                //pass
            }
        }
        bw1.write("\n After " + maxRouteQ + " , " + minRouteQ + " , prediction = " + curQ);
        bw1.write("\n changeCnt = " + changeCnt);
        // filtering: find the path from depot to the destination
        Integer lab;
        int i = 0;
        while ((i < nbroute) && ((lab = P.pollFirst()) != null)) {
            label s = labels.get(lab);
            if (!s.dominated) {
                //原来的判断条件 if (/*(i < nbroute / 2) ||*/ (s.cost < -1e-4)) {
                //todo

                if (/*(i < nbroute / 2) ||*/ (s.cost <= curQ)) {
                    // System.out.println(s.cost);
                    route newroute = new route();
                    newroute.setPr(s.Pr);
                    newroute.setcost(s.cost_noproba);
                    newroute.addcity(s.city);
                    bw2.write("\n NewRoute = " + s.city + " Pr = "
                            + Arrays.toString(s.Mt_));
                    bw2.flush();
                    int path = s.indexPrevLabel;
                    while (path >= 0) {
                        newroute.addcity(labels.get(path).city);
                        bw2.write("\n Node = " + labels.get(path).city + " Pr = "
                                + Arrays.toString(labels.get(path).Mt_));
                        bw2.flush();
                        path = labels.get(path).indexPrevLabel;
                    }
                    newroute.switchpath();
                    newroute.checkTW(userParam);
                    //新路径 每个点的到达时间 按st 算 以及cost
                    newroute.write2log(bw1);
                    bw1.flush();
                    //把生成的route的每个点的时间的概率都写出来看
                    routes.add(newroute);
                    routeQ = Math.max(routeQ, s.cost_noproba);
                    i++;
                }
            }
        }
    }

    public boolean greater2eq(double a, double b) {
        if (a - b > 1E-6 || (a == b)) {
            return true;
        } else {
            return false;
        }
    }
}
