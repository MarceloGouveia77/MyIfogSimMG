package org.fog.vmmigration;

import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.core.SimEntity;
import org.cloudbus.cloudsim.core.SimEvent;
import org.fog.entities.FogDevice;
import org.fog.entities.MobileDevice;
import org.fog.vmmobile.constants.MobileEvents;
import org.fog.vmmobile.constants.Policies;

public class BeforeMigration_OLD {
//
//	public static boolean dataPepare(int policyReplicaVM, MobileDevice smartThing){/*pensar nos parametros*/
//		if(replicaVM(policyReplicaVM)){
//			return containerVM(smartThing);
//		}
//		else{
//			return completeVM(smartThing);
//		}
//	}
//
//	public static boolean replicaVM(int policyReplicaVM){ 
//
//		if(policyReplicaVM == Policies.MIGRATION_CONTAINER_VM){// the simplest 
//			return true;// The migration will be only the Container 
//		}
//		else{
//			return false;// The migration will be all VM
//		}
//	}
//	//	public boolean replicaVM(FogDevice ServerCloudlet){
//	//		It can verify if the VM exist in the next ServerCloudlet
//	//	} 
//
//	public static boolean openConnection(FogDevice sourceServerCloudlet, FogDevice destinationServerCloudlet){
//		//to do it
//		return true;
//	}
//
//	public static boolean completeVM(MobileDevice smartThing){
//		//to do it -> It should migrate the complete VM to next ServerCloudlet		
//		for(int i = 0; i<3 ; i++){
//			if(openConnection(smartThing.getSourceServerCloudlet(), smartThing.getDestinationServerCloudlet())){
////				startMigration(smartThing);
//				return true;
//			}
//			else{
//				//maybe to exchange anything (e.g Links)
//			}
//		}
//		return false;
//			
//	}
//	
//	public static boolean containerVM(MobileDevice smartThing){
//		//to do it -> It should migrate only the Container		
//		for(int i = 0; i<3 ; i++){
//			if(openConnection(smartThing.getSourceServerCloudlet(), smartThing.getDestinationServerCloudlet())){
//				
//				return true;
//			}
//			else{
//				//maybe to exchange anything (e.g Links)
//			}
//		}
//		return false;
//
//	}
////	public static void startMigration(MobileDevice smartThing) {
////		// TODO Auto-generated method stub
////		sendNow(smartThing.getSourceServerCloudlet().getId(), MobileEvents.START_MIGRATION,smartThing);
////		
////	
////	}
//	
}
