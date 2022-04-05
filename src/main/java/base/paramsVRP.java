package base;

// this class contains the inputs and methods to read the inputs
// for the Branch and Price CVRP with TW
// ...I'm afraid that it is not pure OO code
// ...but it is not so bad

import java.io.*;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Random;
import java.util.StringTokenizer;

import javax.security.auth.login.Configuration;


public class paramsVRP {
    String param_file = "./dataset/VrpParam.txt";
    FileWriter fw1 = new FileWriter(makefile(param_file), false);
    BufferedWriter bw1 = new BufferedWriter(fw1);

    public boolean verbose;
    public int rndseed, mvehic, nbclients, capacity;

    public double[][] cost; // for the SPPRC subproblem

    public double[][] distBase; // original distances for the Branch and Bound
    public double[][] dist; //  cij distances that will be updated during the B&B before being used in the CG & SPPRC
    public double[][] ttime;// origin ttime
    public double[][] edges; // weight of each edge during branch and bound
    public double[] posx, posy, d, wval;
    public int[][] shrink;
    public double maxtime, verybig, speed, gap, maxlength;
    public boolean serviceInTW, debug = true;
    String[] citieslab, citiesloc;

    public double[] dual_price;
    public int[] a; // time windows: a=early, b=late, s=service
    public int[] b;
    public int[] s;
    public double[] profit;//node profit
    public double[][] ms;//=new double[][];//ms i,k node 和 k 时长
    public ArrayList<Integer>[] mik;//node 0 k 1,2,3 =new ArrayList<Integer>[];
    public double[] smax;

    public double penalty_value = 0.0;
    public double vehicle_number = 25;

    //param
    public double costWeight = 1.0;
    public double profitWeight = 1.0;
    public double feasible_pro = 0.95;

    public double minPath = Double.MAX_VALUE;
    public double maxPath = 0;
    public Random r = new Random();

    public int maxcnt = 10;


    //public hashMap<Integer,>[][] ms;//=new double[][];//ms i,k node 和 k 时长
    public paramsVRP(int nodenum, int maxcnt) throws IOException {
        gap = 0.00000000001;
        serviceInTW = false;
        nbclients = nodenum;
        speed = 1;
        mvehic = 0;
        verybig = 1E10;
        this.maxcnt = maxcnt;
    }

    public void initParams(String inputPath,double costWeight,double profitWeight) throws IOException {
        int i, j;
        this.costWeight=costWeight;
        this.profitWeight=profitWeight;
        try {
            /**
             * @update 2013. 6. 12
             * @modify Geunho Kim
             *
             *  for Hadoop distributed file system
             */

            BufferedReader br = new BufferedReader(new FileReader(inputPath));

            String line = new String();

            // //////////////////////////
            // for local file system
            // BufferedReader br = new BufferedReader(new FileReader(inputPath));

            for (i = 0; i < 2; i++)
                line = br.readLine();

            String[] tokens = line.split("\\s+");
            mvehic = Integer.parseInt(tokens[0]);
            capacity = Integer.parseInt(tokens[1]);

            citieslab = new String[nbclients + 2];
            d = new double[nbclients + 2];
            a = new int[nbclients + 2];
            b = new int[nbclients + 2];
            s = new int[nbclients + 2];
            posx = new double[nbclients + 2];
            posy = new double[nbclients + 2];
            distBase = new double[nbclients + 2][nbclients + 2];
            cost = new double[nbclients + 2][nbclients + 2];
            dist = new double[nbclients + 2][nbclients + 2];
            ttime = new double[nbclients + 2][nbclients + 2];
            dual_price = new double[nbclients + 2];
            profit = new double[this.nbclients + 2];
            profit[0] = 0.0;
            profit[this.nbclients + 1] = 0.0;

            for (i = 0; i < nbclients + 1; i++) {
                line = br.readLine();
                //System.out.println(line);
                tokens = line.split("\\s+");
                citieslab[i] = tokens[0]; // customer number
                posx[i] = Double.parseDouble(tokens[1]); // x coordinate
                posy[i] = Double.parseDouble(tokens[2]); // y coordinate
                d[i] = Double.parseDouble(tokens[3]); // demand
                a[i] = Integer.parseInt(tokens[4]); // ready time
                b[i] = Integer.parseInt(tokens[5]); // due time
                s[i] = Integer.parseInt(tokens[6]); // service
                profit[i] = Integer.parseInt(tokens[7]);
                //System.err.println(i+" "+b[i]);
                // check if the service should be done before due time
                if (serviceInTW)
                    b[i] -= s[i];
            }
            br.close();

            // second depot : copy of the first one for arrival
            citieslab[nbclients + 1] = citieslab[0];
            d[nbclients + 1] = 0.0;
            a[nbclients + 1] = a[0];
            b[nbclients + 1] = b[0];
            s[nbclients + 1] = 0;
            posx[nbclients + 1] = posx[0];
            posy[nbclients + 1] = posy[0];

            // ---- distances
            maxlength = 0.0;
            for (i = 0; i < nbclients + 2; i++) {
                for (j = 0; j < nbclients + 2; j++) {
                    //todo 距离和服务时间有关系 暂时用int double的话 需要 将double转下标
                    distBase[i][j] = (int) (Math.sqrt((posx[i] - posx[j]) * (posx[i] - posx[j])
                            + (posy[i] - posy[j]) * (posy[i] - posy[j])));
                    // truncate to get the same results as in Solomon
                    //bw1.write("\n distbase,(i,j) " + i + "," + j + " = " + distBase[i][j]);
                    if (maxPath < distBase[i][j])
                        maxPath = distBase[i][j];
                    if (distBase[i][j] > 0 && minPath > distBase[i][j])
                        minPath = distBase[i][j];
                }
                maxlength += maxPath; // a route with a length longer than this is not
                // possible (we need it to check the feasibility of
                // the Column Gen sol.
            }
            for (i = 0; i < nbclients + 2; i++) {
                distBase[i][0] = verybig;
                distBase[nbclients + 1][i] = verybig;
                distBase[i][i] = verybig;
            }

            for (i = 0; i < nbclients + 2; i++)
                for (j = 0; j < nbclients + 2; j++) {
                    dist[i][j] = distBase[i][j];
                }

            // ---- time
            for (i = 0; i < nbclients + 2; i++)
                for (j = 0; j < nbclients + 2; j++)
                    ttime[i][j] = distBase[i][j] / speed;

            for (j = 0; j < nbclients + 2; j++) {
                cost[0][j] = dist[0][j];
                cost[j][nbclients + 1] = dist[j][nbclients + 1];
            }


        } catch (IOException e) {
            System.err.println("Error: " + e);
        }

        wval = new double[nbclients + 2];
        for (i = 1; i < nbclients + 2; i++)
            wval[i] = 0.0;

        edges = new double[nbclients + 2][nbclients + 2];

    }

    //todo
    public void setProfit() throws IOException {
        this.profit = new double[this.nbclients + 2];
        //[L,U] minPath n/2*maxPath
        double Ub = maxPath;
        bw1.write("\n MaxPath = " + maxPath + " , MinPath = " + minPath);
        bw1.flush();
        for (int i = 0; i < this.nbclients + 2; i++) {
            this.profit[i] = minPath + 2 * r.nextInt((int) (Ub - minPath));//todo profitParam
            this.profit[i] = 20.0;
        }
        this.profit[0] = 0.0;
        this.profit[this.nbclients + 1] = 0.0;
        for (int i = 0; i < this.nbclients + 2; i++) {
            bw1.write("\n Node " + i + " profit = " + this.profit[i]);
            bw1.flush();
        }
    }

    //todo
    public void setServiceTime(int a, int b, int c) throws IOException {
        //service time
        // public double[][] ms;//=new double[][];//ms i,k node 和 k 时长 的概率是多少
        // public ArrayList<Integer>[] mik;//node 有时长哪些值
        // public double[] smax;最长服务时间

        mik = new ArrayList[nbclients + 2];
        ms = new double[nbclients + 2][c + 1];//smax
        smax = new double[nbclients + 2];
        for (int i = 1; i < nbclients + 1; i++) {
            bw1.write("\n Node " + i);
            bw1.flush();
            /*ArrayList<Integer> tmp = new ArrayList<Integer>();
            for (int i1 = a; i1 <= c; i1++) {
                tmp.add(i1);
            }
            mik[i] = tmp;
            bw1.write("\n contains = " + tmp.toString());
            //prob
            ms[i][a] = 1.0;*/

            ArrayList<Integer> tmp = new ArrayList<Integer>();
            for (int i1 = a; i1 <= c; i1++) {
                tmp.add(i1);
            }
            mik[i] = tmp;
            for (int i1 = a; i1 <= b; i1++) {
                double prob = 2.0 * (i1 - a) / ((c - a) * (b - a));
                ms[i][i1] = prob;
            }
            for (int i1 = b; i1 <= c; i1++) {
                double prob = 2.0 * (c - i1) / ((c - a) * (c - b));
                ms[i][i1] = prob;
            }
            bw1.write("\n contains = " + tmp.toString());
            bw1.write("\n prob ");
            for (int i1 = a; i1 <= c; i1++) {
                bw1.write("\n " + i1 + " = " + ms[i][i1]);
            }
            bw1.flush();
            //smax
            smax[i] = c;
        }

        //set depot
        ArrayList<Integer> tmp = new ArrayList<Integer>();
        tmp.add(0);
        mik[0] = tmp;
        ms[0][0] = 1.0;
        smax[0] = 0;

        ArrayList<Integer> tmp1 = new ArrayList<Integer>();
        tmp1.add(0);
        mik[nbclients + 1] = tmp1;
        ms[nbclients + 1][0] = 1.0;
        smax[nbclients + 1] = 0;

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