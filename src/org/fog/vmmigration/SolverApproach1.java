package org.fog.vmmigration;

import ilog.concert.*;
import ilog.cplex.*;
import java.util.List;
import org.fog.entities.FogDevice;
import org.fog.entities.MobileDevice;
import org.w3c.dom.UserDataHandler;

/*
 * first approach: maximizes requests accepted first
 * */
public class SolverApproach1 extends Solver{
	
	// priority of each VM request
	public static double[] p;

	// class constructor
	public SolverApproach1(List<FogDevice> serverCloudlets, List<List<Double>> latencyMatrix, List<MobileDevice> smartThings) {

		super(serverCloudlets, latencyMatrix, smartThings);
		
		p = new double[nApps];
		
		setStartupSettings();
	}

	// set the priority of the VMs requests.
	public static void setStartupSettings() {
		for (int i = 0; i < nApps; i++) {
			p[i] = 1;
		}
	}

	// main method. Here we call CPLEX
	public static int[] solve() {

		// maximizes requests accepted first 
		try {
			// define new model
			IloCplex cplex = new IloCplex();
			// get the time at the start of the benchmark
			double startTime = cplex.getCplexTime();

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
			IloLinearNumExpr objective = cplex.linearNumExpr();
			for (int i = 0; i < nApps; i++) {
				for (int j = 0; j < nCloudlets; j++) {
					objective.addTerm(p[i], pm[i][j]);
				}
			}

			// define first objective (Equation 1)
			cplex.addMaximize(objective);

			// constraints
			// equation 3
			for (int i = 0; i < nApps; i++) {
				IloLinearNumExpr expr = cplex.linearNumExpr();
				for (int j = 0; j < nCloudlets; j++) {
					expr.addTerm(1, pm[i][j]);
				}
				cplex.addLe(expr, 1.0);
			}

			for (int j = 0; j < nCloudlets; j++) {
				cplex.addLe(usedCPU[j], availableCPU[j]);// equation 4
				cplex.addLe(usedRAM[j], availableRAM[j]);// equation 5
				cplex.addLe(usedBandwidth[j], availableBandwidth[j]);// equation 6
				cplex.addLe(usedStorage[j], availableStorage[j], "Storage");// equation 7
			}

			 cplex.setParam(IloCplex.Param.Simplex.Display, 0);
			 
			 cplex.exportModel("model.lp");

			// solves the first objective function - Equation 1
			if (cplex.solve()) {
				printResults(false, cplex, pm, startTime);

				// starts the second optimization. Minimizes the latency based on the previous results (Equation 2)
				IloCplex cplex2 = new IloCplex();
				IloNumVar[][] pm2 = new IloNumVar[nApps][];
				for (int i = 0; i < nApps; i++) {
					pm2[i] = cplex2.boolVarArray(nCloudlets);
				}
				IloLinearNumExpr[] usedRAM2 = new IloLinearNumExpr[nCloudlets];
				IloLinearNumExpr[] usedCPU2 = new IloLinearNumExpr[nCloudlets];
				IloLinearNumExpr[] usedBW2 = new IloLinearNumExpr[nCloudlets];
				IloLinearNumExpr[] usedStorage2 = new IloLinearNumExpr[nCloudlets];

				for (int j = 0; j < nCloudlets; j++) {
					usedRAM2[j] = cplex2.linearNumExpr();
					usedCPU2[j] = cplex2.linearNumExpr();
					usedBW2[j] = cplex2.linearNumExpr();
					usedStorage2[j] = cplex2.linearNumExpr();

					for (int i = 0; i < nApps; i++) {
						usedRAM2[j].addTerm(requiredRAM[i], pm2[i][j]);
						usedCPU2[j].addTerm(requiredCPU[i], pm2[i][j]);
						usedBW2[j].addTerm(requiredBandwidth[i], pm2[i][j]);
						usedStorage2[j].addTerm(requiredStorage[i], pm2[i][j]);
					}
				}
				IloLinearNumExpr objective2 = cplex2.linearNumExpr();
				for (int i = 0; i < nApps; i++) {
					for (int j = 0; j < nCloudlets; j++) {
						objective2.addTerm(latency[i][j], pm2[i][j]); //
					}
				}
				
				// define second objective function (Equation 2)
				cplex2.addMinimize(objective2);

				// equation 3
				for (int i = 0; i < nApps; i++) {
					IloLinearNumExpr expr = cplex2.linearNumExpr();
					for (int j = 0; j < nCloudlets; j++) {
						expr.addTerm(1, pm2[i][j]);
					}
					cplex2.addLe(expr, 1.0);
				}

				for (int j = 0; j < nCloudlets; j++) {
					cplex2.addLe(usedCPU2[j], availableCPU[j]);// Equation 4
					cplex2.addLe(usedRAM2[j], availableRAM[j]);// Equation 5
					cplex2.addLe(usedBW2[j], availableBandwidth[j]);// Equation 6
					cplex2.addLe(usedStorage2[j], availableStorage[j]);// Equation 7
				}

				//ensure that an application allocated by the fist objective function will be allocated by second objective function
				for (int i = 0; i < nApps; i++) {
					IloLinearNumExpr expr = cplex2.linearNumExpr();
					int sumApps = 0;
					for (int j = 0; j < nCloudlets; j++) {
						expr.addTerm(1, pm2[i][j]);
						if (cplex.getValue(pm[i][j]) > 0.0) {
							sumApps++;
						}
					}
					cplex2.addEq(expr, sumApps);
				}

				 cplex2.setParam(IloCplex.Param.Simplex.Display, 0);

				// solves the second objective function
				if (cplex2.solve()) {
					// display and saves the ILP's data
					printResults(false, cplex2, pm2, startTime);
					// get the final VM placement
					int result[] = getResult(cplex2, pm2);
					cplex2.end();
					cplex.end();
					return result;
				}
				else {
					System.out.println("problem not solved");
					return null;
				}
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
