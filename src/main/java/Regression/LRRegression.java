package Regression;

import org.junit.Test;
import weka.classifiers.Evaluation;
import weka.classifiers.functions.LinearRegression;
import weka.clusterers.EM;
import weka.clusterers.SimpleKMeans;
import weka.core.*;
import weka.core.converters.ArffLoader;
import weka.core.converters.ConverterUtils.DataSource;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LRRegression {

    public LRRegression() {

    }

    /**
     * 对数据集合做线性回归，从而挖掘出其中的公式
     *
     * @return 线性回归运算得到的公式，以及运算结果的评估
     * @throws Exception
     */
    public LinearRegression doLinearRegression(ArrayList<double[]> dual_prices, ArrayList<Double> Qs) throws Exception {

        // 获取训练数据集
        ArrayList<Attribute> attributes = new ArrayList<>();
        for (int j = 1; j <= 25; j++) {
            attributes.add(new Attribute("dual" + Integer.toString(j)));
        }
        attributes.add(new Attribute("Qvalue"));

        //set instances
        Instances insTrain = new Instances("customers", attributes, 0);
        for (int i = 0; i < dual_prices.size(); i++) {
            Instance instance = new DenseInstance(attributes.size());
            for (int j = 0; j < dual_prices.get(i).length; j++) {
                instance.setValue(j, dual_prices.get(i)[j]);
            }
            instance.setValue(25, Qs.get(i));
            insTrain.add(instance);
        }

        // 设置训练集中，target的index 设置分类属性所在行号（第一行为0号）
        insTrain.setClassIndex(insTrain.numAttributes() - 1);
        // 定义分类器的类型 , 我们采用线性回归
        LinearRegression lr = new LinearRegression();
        // 训练分类器
        lr.buildClassifier(insTrain);
        // 评估线性回归的结果
        Evaluation eval = new Evaluation(insTrain);
        double[] doubles = eval.evaluateModel(lr, insTrain);// 评估结果(返回的是对每次训练数据的Y值)
        // 构造结果对象
        StringBuilder sb = new StringBuilder();
        sb.append("机器学习后产生的线性回归公式:\n" + lr.toString() + "\n\n");
        sb.append("评估此结果:" + eval.toSummaryString() + "\n");
        System.out.println(sb.toString());
        //回去模型系数（各项占比w以及误差项b）
        double[] coefficients = lr.coefficients();
        System.out.println("系数:" + Arrays.toString(coefficients));//出现0只有两种可能，Y值为0(倒数第二项)，X中不影响（或影响很小）也可能为0，最后一项为b误差项
        System.out.println("模型使用多少个系数：" + lr.numParameters());
        System.out.println("" + eval.totalCost());
        System.out.println("相关系数Correlation coefficient " + eval.correlationCoefficient());
        System.out.println("平均绝对误差Mean absolute error " + eval.meanAbsoluteError());
        System.out.println("均方根误差(标准差【小写西格玛o】)Root mean squared error " + eval.rootMeanSquaredError());
        System.out.println("相对绝对误差Relative absolute error  " + eval.relativeAbsoluteError());
        System.out.println("根相对平方误差Root relative squared error " + eval.rootRelativeSquaredError());
        System.out.println("实例总数Total Number of Instances " + insTrain.size()); //insTrain.numAttributes()获取指标数【从1开始要减1】
        return lr;
    }

    /***
     * @param list  真实数据
     * @param lr    训练数据的线性模型
     * @return {@link double} 最终结果Y
     * @data: 2020-10-9 14:50 @author:wzy
     * description: 根据真实数据，预测最终结果（线性回归）
     */
    public double countLinerRegression(double[] list, LinearRegression lr) {
        double result = 0;
        //构造线性回归实例对象
        SparseInstance ins = new SparseInstance(list.length);
        for (int i = 0; i < list.length; i++) {
            ins.setValue(i, list[i]);
        }
        try {
            //使用此模型根据给定的参数得到预测的结果值
            result = lr.classifyInstance(ins);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return result;
    }

    @Test
    public void test() throws Exception {
        LRRegression LR = new LRRegression();
        double[] dual1 = new double[]{1.0, 2.0, 3.0, 2.0};
        double Q1 = 6;
        ArrayList<double[]> a = new ArrayList<>();
        a.add(dual1);
        ArrayList<Double> b = new ArrayList<>();
        b.add(Q1);
        LinearRegression lr = LR.doLinearRegression(a, b);
        double[] testList = new double[]{0.5, 1.0, 1.5, 1.0};
        double result = LR.countLinerRegression(testList, lr);
        System.out.println("Result: "+result);
    }

}
