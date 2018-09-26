package org.fog.vmmigration;

import ilog.concert.*;
import ilog.cplex.*;
import ilog.cplex.IloCplex.UnknownObjectException;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.List;
import org.fog.entities.FogDevice;
import org.fog.entities.MobileDevice;

/*
 * Parent class in the ILP package.
 * 
 * Solver class has the common attributes and methods among the approaches
 * */
public class Solver {
	
	public static final int APPROACH_1 = 1;
	public static final int APPROACH_2 = 2;

	// input size
	public static int nApps; //number of users' VMs
	public static int nCloudlets; // number of available cloudlets

	// resource required
	public static int[] requiredRAM; // in Megabytes
	public static double[] requiredCPU; // in MIPS (Millions Instructions Per Second)
	public static long[] requiredBandwidth;// in Megabits
	public static long[] requiredStorage; // in Megabytes

	// resource available
	public static int[] availableRAM;
	public static double[] availableCPU;
	public static long[] availableBandwidth;
	public static long[] availableStorage;

	// expressions
	public static IloLinearNumExpr[] usedRAM;
	public static IloLinearNumExpr[] usedCPU;
	public static IloLinearNumExpr[] usedBandwidth;
	public static IloLinearNumExpr[] usedStorage;

	// target matrix. Values come from IFogSim
	public static double[][] latency;

	// list of objects from IFogSim
	public static List<FogDevice> cloudlets;
	public static List<List<Double>> latencyMatrix;
	public static List<MobileDevice> smartThings;

	// class constructor 
	public Solver(List<FogDevice> serverCloudlets, List<List<Double>> latencyMatrix, List<MobileDevice> smartThings) {

		this.cloudlets = serverCloudlets;
		this.latencyMatrix = latencyMatrix;
		this.smartThings = smartThings;
		nApps = smartThings.size();
		
		// input size
		nCloudlets = cloudlets.size(); // cloudlets

		// resource required
		requiredRAM = new int[nApps];
		requiredCPU = new double[nApps];
		requiredBandwidth = new long[nApps];
		requiredStorage = new long[nApps];

		// resource available
		availableRAM = new int[nCloudlets];
		availableCPU = new double[nCloudlets];
		availableBandwidth = new long[nCloudlets];
		availableStorage = new long[nCloudlets];

		usedRAM = new IloLinearNumExpr[nCloudlets];
		usedCPU = new IloLinearNumExpr[nCloudlets];
		usedBandwidth = new IloLinearNumExpr[nCloudlets];
		usedStorage = new IloLinearNumExpr[nCloudlets];

		// target matrix
		latency = new double[nApps][nCloudlets];

		// initialize values
		setStartSettings();
	}

	// initializes values of object's attributes. Values come from IFogSim 
	public static void setStartSettings() {
		// setting resources available
		for (int i = 0; i < nCloudlets; i++) {
			availableRAM[i] = cloudlets.get(i).getHost().getRamProvisioner().getAvailableRam();
			availableCPU[i] = cloudlets.get(i).getHost().getAvailableMips();
			availableBandwidth[i] = cloudlets.get(i).getHost().getBwProvisioner().getAvailableBw();
			availableStorage[i] = cloudlets.get(i).getHost().getStorage();
		}

		// setting resources required;
		for (int i = 0; i < nApps; i++) {
			requiredRAM[i] = smartThings.get(i).getVmMobileDevice().getCurrentRequestedRam();
			requiredCPU[i] = smartThings.get(i).getVmMobileDevice().getCurrentRequestedMaxMips();
			requiredBandwidth[i] = smartThings.get(i).getVmMobileDevice().getCurrentRequestedBw();
			requiredStorage[i] = smartThings.get(i).getVmMobileDevice().getSize();
		}

		// setting latency values.
		for (int i = 0; i < nApps; i++) {
			List<Double> lm = latencyMatrix.get(i);
			for (int j = 0; j < nCloudlets; j++) {
				latency[i][j] = lm.get(j);
			}
		}
	}

	// saves the ILP result in a text file
	public static void saveResults(int approach, IloCplex cplex, double start, int appScheduled, double totalLatency, int ram, int cpu, int bandwidth, int storage) throws IloException {
		try (FileWriter fw = new FileWriter(approach + "_approach_results.csv", true);
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter out = new PrintWriter(bw)) {
			out.println(nCloudlets + "\t" +
						nApps + "\t" +
						cplex.getObjValue() + "\t" +
						(cplex.getCplexTime() - start) + "\t" +
						appScheduled + "\t" +
						totalLatency + "\t" +
						(totalLatency / appScheduled) + "\t" +
						ram + "\t" +
						bandwidth + "\t" +
						storage);
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	// displays the ILP results using console and calls saveResults
	public static void printResults(Boolean printFullLog, IloCplex cplex, IloNumVar[][] placement_matrix, double start) throws IloException {
		int ram = 0;
		int cpu = 0;
		int bw = 0;
		int storage = 0;
		int appScheduled = 0;
		double totalLatency = 0;
		// solve model
		if(printFullLog){
			System.out.println("obj = " + cplex.getObjValue());
			System.out.println("time = " + (cplex.getCplexTime() - start));
		}
		for (int j = 0; j < nCloudlets; j++) {
			if(printFullLog){
				System.out.println(j);
				System.out.println("RAM = " + cplex.getValue(usedRAM[j]));
				System.out.println("CPU = " + cplex.getValue(usedCPU[j]));
				System.out.println("BW in = " + cplex.getValue(usedBandwidth[j]));
				System.out.println("BW out = " + cplex.getValue(usedStorage[j]));
			}
			ram += cplex.getValue(usedRAM[j]);
			cpu += cplex.getValue(usedCPU[j]);
			bw += cplex.getValue(usedBandwidth[j]);
			storage += cplex.getValue(usedStorage[j]);
		}
		for (int i = 0; i < nApps; i++) {
			for (int j = 0; j < nCloudlets; j++) {
				if(printFullLog){
					System.out.print(cplex.getValue(placement_matrix[i][j]) + " ");
				}
				if (cplex.getValue(placement_matrix[i][j]) == 1) {
					appScheduled++;
					totalLatency += latency[i][j];
				}
			}
			if(printFullLog){
				System.out.println("");
			}
		}
		System.out.println("Apps scheduled = " + appScheduled + "/" + nApps);
		System.out.println("Total Latency  = " + totalLatency + " mean = " + totalLatency / appScheduled);
		if(printFullLog){
			double sumResource=0;
			for (int j = 0; j < nCloudlets; j++) {
				sumResource += availableRAM[j];
			}
			System.out.println("RAM = " + ram + "/" + sumResource);
			sumResource=0;
			for (int j = 0; j < nCloudlets; j++) {
				sumResource += availableCPU[j];
			}
			System.out.println("CPU = " + cpu + "/" + sumResource);
			sumResource=0;
			for (int j = 0; j < nCloudlets; j++) {
				sumResource += availableBandwidth[j];
			}
			System.out.println("bw = " + bw + "/" + sumResource);
			sumResource=0;
			for (int j = 0; j < nCloudlets; j++) {
				sumResource += availableRAM[j];
			}
			System.out.println("storage = " + storage + "/" + sumResource);
		}

		saveResults(1, cplex, start, appScheduled, totalLatency, ram, cpu, bw, storage);
	}
	
	// returns the final VM placement
	public static int[] getResult(IloCplex cplex, IloNumVar[][] placement_matrix) throws UnknownObjectException, IloException{
		int result[] = new int[nApps];
		for (int i = 0; i < nApps; i++) {
			result[i] = -1;
			for (int j = 0; j < nCloudlets; j++) {
				if (cplex.getValue(placement_matrix[i][j]) == 1) {
					result[i] = j;
					break;
				}
			}
		}
		return result;
	}
}
