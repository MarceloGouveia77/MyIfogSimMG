package org.fog.vmmigration;

import ilog.concert.*;
import ilog.cplex.*;
import java.util.List;

import org.fog.entities.FogDevice;
import org.fog.entities.MobileDevice;

/*
 * second approach: minimizes latency first
 * */
public class SolverApproach2 extends Solver{

	// inverse latency matrix. Lower latency have higher priority
	public static double[][] latencyInv;

	// class constructor
	public SolverApproach2(List<FogDevice> serverCloudlets, List<List<Double>> latencyMatrix, List<MobileDevice> smartThings) {

		super(serverCloudlets, latencyMatrix, smartThings);
		
		latencyInv = new double[nApps][nCloudlets];
		
		setStartupSettings();
	}
	
	// initializes attributes.
	public static void setStartupSettings() {

		// setting the inverse latency matrix. Lower latency become the higher priority in the latencyInv
		double max = latency[0][0];
		for (int i = 0; i < nApps; i++) {
			for (int j = 0; j < nCloudlets; j++) {
				if (max < latency[i][j]) {
					max = latency[i][j];
				}
			}
		}

		for (int i = 0; i < nApps; i++) {
			for (int j = 0; j < nCloudlets; j++) {
				latencyInv[i][j] = (max - latency[i][j]) + 1;
//				System.out.print(latencyInv[i][j] + " ");
			}
//			System.out.println("");
		}
	}

	// main method. Here we call CPLEX
	public static int[] solve() {

		// minimizes latency first
		try {
			// define new model
			IloCplex cplex = new IloCplex();
			double start = cplex.getCplexTime();
			
			// setting the cplex variables
			// placement matrix
			IloNumVar[][] pm = new IloNumVar[nApps][];
			for (int i = 0; i < nApps; i++) {
				pm[i] = cplex.boolVarArray(nCloudlets);
			}

			for (int j = 0; j < nCloudlets; j++) {
				usedRAM[j] = cplex.linearNumExpr();
				usedCPU[j] = cplex.linearNumExpr();
				usedBandwidth[j] = cplex.linearNumExpr();
				usedStorage[j] = cplex.linearNumExpr();

				for (int i = 0; i < nApps; i++) {
					usedRAM[j].addTerm(requiredRAM[i], pm[i][j]);
					usedCPU[j].addTerm(requiredCPU[i], pm[i][j]);
					usedBandwidth[j].addTerm(requiredBandwidth[i], pm[i][j]);
					usedStorage[j].addTerm(requiredStorage[i], pm[i][j]);
				}
			}
			
			//defines the cplex objective function
			IloLinearNumExpr objective = cplex.linearNumExpr();
			for (int i = 0; i < nApps; i++) {
				for (int j = 0; j < nCloudlets; j++) {
					objective.addTerm(latencyInv[i][j], pm[i][j]);
				}
			}

			cplex.addMaximize(objective);

			// constraints
			// equation 5
			for (int i = 0; i < nApps; i++) {
				IloLinearNumExpr expr = cplex.linearNumExpr();
				for (int j = 0; j < nCloudlets; j++) {
					expr.addTerm(1, pm[i][j]);
				}
				cplex.addLe(expr, 1.0);
			}

			for (int j = 0; j < nCloudlets; j++) {
				cplex.addLe(usedCPU[j], availableCPU[j]);// equation 6
				cplex.addLe(usedRAM[j], availableRAM[j]);// equation 7
				cplex.addLe(usedBandwidth[j], availableBandwidth[j]);// equation 8
				cplex.addLe(usedStorage[j], availableStorage[j]);// equation 9
			}

			cplex.setParam(IloCplex.Param.Simplex.Display, 0);

			// solve model
			if (cplex.solve()) {
				// display and saves the ILP's data
				printResults(false, cplex, pm, start);
				// get the final VM placement
				int result[] = getResult(cplex, pm);
				cplex.end();
				return result;
			}
			else {
				System.out.println("problem not solved");
				return null;
			}
		} catch (IloException exc) {
			exc.printStackTrace();
		}
		return null;
	}
}
