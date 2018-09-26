package org.fog.vmmigration;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import javax.print.attribute.standard.Severity;

import org.cloudbus.cloudsim.NetworkTopology;
import org.fog.entities.*;
import org.fog.localization.Coordinate;
import org.fog.localization.DiscoverLocalization;
import org.fog.localization.Distances;
import org.fog.vmmobile.constants.*;
import org.fog.localization.Coordinate;


public class Migration {


	private static boolean migrationPoint;
	private static boolean migrationZone;
	private int location;
	private ApDevice correntAP;
	private FogDevice correntServerCloudlet;
	private MobileDevice correntSmartThing;
	private ApDevice apAvailable;
	private FogDevice serverCloudletAvailable;
	private int flowDirection;
	private static List<ApDevice> apsAvailable;
	private static List<FogDevice> serverCloudletsAvailable;
	private static int policyReplicaVM;

	/**
	 * @param args
	 * @author Marcio Moraes Lopes
	 */


	public static List<ApDevice> apAvailableList(List<ApDevice> oldApList
			, MobileDevice smartThing ){//It looks to cone and return the Aps available list
		List<ApDevice> newApList =new ArrayList<>();
		//		int stCoordX = smartThing.getCoord().getCoordX();
		//		int stCoordY = smartThing.getCoord().getCoordY();
		//		int apCoordX;
		//		int apCoordY;
		int localAp;
		boolean cone;
		for(ApDevice ap: oldApList){
//			localAp=DiscoverLocalization.discoverLocal(smartThing.getCoord(),ap.getCoord());//return the relative position between Access point and smart thing -> set this value
//			cone=migrationZoneFunction(localAp,smartThing.getDirection());
			//, localAp);
//			if(cone&&(ap.getMyId()!=smartThing.getSourceAp().getMyId())){
//				if(smartThing.getName().equals("SmartThing175")&&ap.getName().equals("AccessPoint138")){
//					System.out.print(ap.getName());
//					System.out.println(" - LocalAp tem que ser 3, 4 ou 5: "+localAp);
//				}
			if(!smartThing.getSourceAp().equals(ap))
				newApList.add(ap);
//			}
		}

		return newApList;



		//		for(ApDevice ap:oldApList){
		//			apCoordX = ap.getCoord().getCoordX();
		//			apCoordY = ap.getCoord().getCoordY();
		//			if(smartThing.getDirection()==Directions.EAST){
		//				if(apCoordX > stCoordX){ //180 degrees
		//					newApList.add(ap);
		//				}
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.NORTH){
		//				if(apCoordY > stCoordY){ //180 degrees
		//					newApList.add(ap);
		//				}
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.WEST){
		//				if(apCoordX < stCoordX){ //180 degrees
		//					newApList.add(ap);
		//				}
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.SOUTH){
		//				if(apCoordY < stCoordY){ //180 degrees
		//					newApList.add(ap);
		//				}
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.NORTHEAST){
		//				if(apCoordX > stCoordX && apCoordY > stCoordY){//90 degrees 
		//					newApList.add(ap);
		//				}
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.NORTHWEST){
		//				if(apCoordX < stCoordX && apCoordY > stCoordY){ //90 degrees
		//					newApList.add(ap);					
		//				}
		//
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.SOUTHWEST){
		//				if(apCoordX < stCoordX && apCoordY < stCoordY ){ //90 degrees
		//					newApList.add(ap);					
		//				}
		//				continue;
		//			}
		//			else if(smartThing.getDirection()==Directions.SOUTHEAST){
		//				if(apCoordX > stCoordX && apCoordY < stCoordY ){ //90 degrees
		//					newApList.add(ap);					
		//				}
		//				continue;
		//			}
		//
		//		}
		//
		//		return newApList;
	}

	

	
	

	public static int nextAp(List<ApDevice> apDevices, MobileDevice smartThing){//Policy: the closest Ap 

		setApsAvailable(apAvailableList(apDevices,smartThing));//It return apDevice list without the smartThing's sourceAp 
		if (getApsAvailable().size() == 0){
			return -1;
		}
		//System.out.println("Aps: "+getApsAvailable());
		//		ApDevice ap=null;
		//		for(int i = 0;i < getApsAvailable().size();i++){
		//			ap=getApsAvailable().get(i);
		//			if(ap.getMyId() == smartThing.getSourceAp().getMyId()){
		//				getApsAvailable().remove(i);
		//				
		//			}
		//		}
		////		if (getApsAvailable().size() == 0){
		////			return -1;
		////		}
		return  Distances.theClosestAp(getApsAvailable(), smartThing);//return the closest ap's id or -1 if it doesn't exist
	}
	public int nextApFromCloudlet(Set<ApDevice> apDevices, MobileDevice smartThing){

		return 0;
	}
	public static boolean insideCone(int smartThingDirection, int zoneDirection){//
		int ajust1, ajust2;

		if(smartThingDirection==Directions.EAST){
			ajust1=Directions.SOUTHEAST;
			ajust2=Directions.EAST+1;
		}
		else if(smartThingDirection==Directions.SOUTHEAST){
			ajust1=Directions.SOUTHEAST-1;
			ajust2=Directions.EAST;
		}
		else{
			ajust1=smartThingDirection-1; /*plus 45 degree*/
			ajust2=smartThingDirection+1;
		}

		if(zoneDirection == smartThingDirection || 
				zoneDirection==ajust1 || 
				zoneDirection == ajust2) /*Define Migration Zone -> it looks for 135 degree = 45 way + 45 way1 +45 way2*/
			return true;
		else
			return false;
	}
	
	public static List<FogDevice> serverClouletsAvailableList(List<FogDevice> oldServerCloudlets, MobileDevice smartThing){
		ArrayList<String[]> path = smartThing.getPath();
		int travelTimeId = smartThing.getTravelTimeId() + smartThing.getTravelPredicTime();
        if(travelTimeId >= path.size()){
        	travelTimeId = path.size()-1;
        }
        
        String[] coodinates = path.get(travelTimeId);

        int direction = Coordinate.convertDirection(Double.parseDouble(coodinates[1]));
        int x = (int) Double.parseDouble(coodinates[2]);
        int y = (int) Double.parseDouble(coodinates[3]);

        smartThing.setFutureCoord(x, y);
	
		List<FogDevice> newServerCloudlets = new ArrayList<>();

		int localServerCloudlet;
		boolean cone;
		for(FogDevice sc: oldServerCloudlets ){
			localServerCloudlet=DiscoverLocalization.discoverLocal(
					smartThing.getFutureCoord(),sc.getCoord());//return the relative position between Server Cloudlet and smart thing -> set this value
			cone = insideCone(localServerCloudlet,direction);
			//, localAp);
			if(cone&&(sc.getMyId()!=smartThing.getSourceServerCloudlet().getMyId())){
				newServerCloudlets.add(sc);
			}
		}
		return newServerCloudlets;




		//		
		//
		//		for(FogDevice sc:oldServerCloudlets){
		//			if(sc.isAvailable()){				
		//
		//				scCoordX = sc.getCoord().getCoordX();
		//				scCoordY = sc.getCoord().getCoordY();
		//				if(smartThing.getDirection()==Directions.EAST){
		//					if(scCoordX > stCoordX){ //180 degrees
		//						newServerCloudlets.add(sc);
		//					}
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.NORTH){
		//					if(scCoordY > stCoordY){ //180 degrees
		//						newServerCloudlets.add(sc);
		//					}
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.WEST){
		//					if(scCoordX < stCoordX){ //180 degrees
		//						newServerCloudlets.add(sc);
		//					}
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.SOUTH){
		//					if(scCoordY < stCoordY){ //180 degrees
		//						newServerCloudlets.add(sc);
		//					}
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.NORTHEAST){
		//					if(scCoordX > stCoordX && scCoordY > stCoordY){//90 degrees 
		//						newServerCloudlets.add(sc);
		//					}
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.NORTHWEST){
		//					if(scCoordX < stCoordX && scCoordY > stCoordY){ //90 degrees
		//						newServerCloudlets.add(sc);					
		//					}
		//
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.SOUTHWEST){
		//					if(scCoordX < stCoordX && scCoordY < stCoordY ){ //90 degrees
		//						newServerCloudlets.add(sc);					
		//					}
		//					continue;
		//				}
		//				else if(smartThing.getDirection()==Directions.SOUTHEAST){
		//					if(scCoordX > stCoordX && scCoordY < stCoordY ){ //90 degrees
		//						newServerCloudlets.add(sc);					
		//					}
		//					continue;
		//				}
		//
		//			}
		//		}
		//		return newServerCloudlets;
		//
	}
	public static int nextServerCloudlet(List<FogDevice> serverCloudlets, MobileDevice smartThing){//Policy: the closest serverCloudlet

		setServerCloudletsAvailable(serverClouletsAvailableList(serverCloudlets, smartThing));
		if(getServerCloudletsAvailable().size()==0){
			return -1;
		}
		else{
			return Distances.theClosestServerCloudlet(getServerCloudletsAvailable(), smartThing);
		}
	}

	
		public static boolean isEdgeAp(ApDevice apDevice, MobileDevice smartThing){
		if(apDevice.getServerCloudlet().getMyId() == smartThing.getSourceServerCloudlet().getMyId())// verify if the next Ap is edge
			return false; 
		else 
			return true;
	}

	public static int lowestLatencyCostServerCloudlet(List<FogDevice> oldServerCloudlets, List<ApDevice> oldApDevices, MobileDevice smartThing){
		List<FogDevice> newServerCloudlets = new ArrayList<>();
		List<FogDevice> numServerCloudlets = new ArrayList<>();
		List<Double> costList = new ArrayList<>();
		List< List<Double> > latencyMatrix = new ArrayList<>();
		List<MobileDevice> smartThings = new ArrayList<>(); 

		for(FogDevice sc: oldServerCloudlets){ 
			//if(sc.getMyId()!=smartThing.getSourceAp().getMyId()){
				newServerCloudlets.add(sc);
			//}
		}

		for(int i = 0; i<9; i++){
			int destinationServerCloudlet = nextServerCloudlet(newServerCloudlets, smartThing);
			if(destinationServerCloudlet>=0){
				for(FogDevice sc1:newServerCloudlets){
					if(sc1.getMyId()==destinationServerCloudlet){
						numServerCloudlets.add(sc1);
						break;
					}
				}

				FogDevice sc=null;
				for(int j = 0;j < newServerCloudlets.size();j++){

					sc=newServerCloudlets.get(j);
					if(sc.getMyId()==destinationServerCloudlet){
						newServerCloudlets.remove(sc);
						break;
					}
				}
			}
			else {
				break;
			}
		}

		if(numServerCloudlets.size()==0){
			return -1;
		}
		//this point numServerCloudlets has + than 0 and - than 10 sc
		double sumCost;
		int choose =-1;
		double minCost=-1;
		int idNextAp = nextAp(oldApDevices, smartThing);
		
		if(idNextAp<0){
			return -1;
		}
		
		System.out.println("All__________________");
		for(FogDevice sc: oldServerCloudlets){ 
			System.out.println(sumCostFunction(sc,oldApDevices.get(idNextAp),smartThing));
		}
		System.out.println("min1__________________");
		for(int i = 0; i<numServerCloudlets.size();i++){
			minCost = sumCostFunction(numServerCloudlets.get(i),oldApDevices.get(idNextAp),smartThing);
			System.out.println(minCost);
			if(minCost>=0){
				choose=numServerCloudlets.get(i).getMyId();
				break;
			}
		}
		System.out.println("min2__________________");
		if(minCost<0){
			return -1;
		}

		for(FogDevice sc: numServerCloudlets){
			sumCost = sumCostFunction(sc,oldApDevices.get(idNextAp),smartThing);
			costList.add(sumCost);
			System.out.println(sumCost);
			if(sumCost<0){
				continue;
			}
			if(sumCost<minCost){
				minCost = sumCost;
				choose = sc.getMyId();
			}
		}

		return choose;
	}

	public static double sumCostFunction(FogDevice serverCloudlet, ApDevice nextAp, MobileDevice smartThing){
		double sum = -1;
//		int idNextAp;
		List<ApDevice> tempListAps = new ArrayList<>(); // It creates a temporary List to invoke the nextAp
//		for(ApDevice ap: serverCloudlet.getApDevices()){ 
//			tempListAps.add(ap);
//		}
//		if(tempListAps.size()==0){
//			return -1;
//		}
//		idNextAp = nextAp(tempListAps, smartThing);
//		if(idNextAp<0){
//			return -1;
//		}		
//		double testDelay = NetworkTopology.getDelay(nextAp.getServerCloulet().getId(),serverCloudlet.getId());		
	
//	for(ApDevice ap:tempListAps){
		if(nextAp.getServerCloudlet().equals(serverCloudlet)){
			 sum = NetworkTopology.getDelay(smartThing.getId(), nextAp.getId()) 
					 + NetworkTopology.getDelay(nextAp.getId(),nextAp.getServerCloudlet().getId())
					 + (1.0/nextAp.getServerCloudlet().getHost().getAvailableMips())
					 + LatencyByDistance.latencyConnection(nextAp.getServerCloudlet(), smartThing);
					 
					 
//					 nextAp.getUplinkLatency()+nextAp.getServerCloulet().getUplinkLatency()
//					 +(1.0/serverCloudlet.getHost().getAvailableMips());
		}
		else{
			 sum = NetworkTopology.getDelay(smartThing.getId(), nextAp.getId()) 
					 + NetworkTopology.getDelay(nextAp.getId(),nextAp.getServerCloudlet().getId())
					 + 1.0 //router
					 + NetworkTopology.getDelay(nextAp.getServerCloudlet().getId(),serverCloudlet.getId())
					 + (1.0/serverCloudlet.getHost().getAvailableMips())
			 		 + LatencyByDistance.latencyConnection(serverCloudlet, smartThing);
					 
					 
//					 nextAp.getUplinkLatency()+nextAp.getServerCloulet().getUplinkLatency() +
//					 (1.0/serverCloudlet.getHost().getAvailableMips()) +
//					  NetworkTopology.getDelay(nextAp.getServerCloulet().getId(),serverCloudlet.getId());		
		}
//		for(ApDevice ap:tempListAps){
//			if(idNextAp==ap.getMyId()){ 
//				VERIFICAR E PENSAR EM UMA NOVA LOGICA PARA OS AP DEVICES. BASICAMENTE DEVO ESCOLHER SOMENTE O AP MAIS PROXIMO, OU SEJA
//				O AP DO HANDOFF. E INCLUIR NA CONTA:
//					- LATENCIA DO AP DE HANDOFF
//					- LATENCIA DA CLOUDLET DO AP DE HANDOFF
//					- LATENCIA DA REDE DA CLOUDLET DO AP DE HANDOFF E A CLOUDLET DE DESTINO -> NAO SEI SE 
//					- mips
//				sum = ap.getUplinkLatency()+serverCloudlet.getUplinkLatency()+serverCloudlet.getRatePerMips()+
//						smartThing.getVmLocalServerCloudlet().getNetServerCloudlets().get(serverCloudlet).doubleValue();//cost of network
//				break;
//			}
//			else {
//				sum =-1;
//			}
//		}
		return sum;



	}

	public static boolean isMigrationPoint() {
		return migrationPoint;
	}

	public static void setMigrationPoint(boolean migrationPoint) {
		Migration.migrationPoint = migrationPoint;
	}

	public static boolean isMigrationZone() {
		return migrationZone;
	}

	public static void setMigrationZone(boolean migrationZone) {
		Migration.migrationZone = migrationZone;
	}

	public int getLocation() {
		return location;
	}

	public void setLocation(int location) {
		this.location = location;
	}

	public ApDevice getCorrentAP() {
		return correntAP;
	}

	public void setCorrentAP(ApDevice correntAP) {
		this.correntAP = correntAP;
	}

	public FogDevice getCorrentServerCloudlet() {
		return correntServerCloudlet;
	}

	public void setCorrentServerCloudlet(FogDevice correntServerCloudlet) {
		this.correntServerCloudlet = correntServerCloudlet;
	}

	public MobileDevice getCorrentSmartThing() {
		return correntSmartThing;
	}

	public void setCorrentSmartThing(MobileDevice correntSmartThing) {
		this.correntSmartThing = correntSmartThing;
	}

	public ApDevice getApAvailable() {
		return apAvailable;
	}

	public void setApAvailable(ApDevice apAvailable) {
		this.apAvailable = apAvailable;
	}

	public FogDevice getServerCloudletAvailable() {
		return serverCloudletAvailable;
	}

	public void setServerCloudletAvailable(FogDevice serverCloudletAvailable) {
		this.serverCloudletAvailable = serverCloudletAvailable;
	}

	public int getFlowDirection() {
		return flowDirection;
	}

	public void setFlowDirection(int flowDirection) {
		this.flowDirection = flowDirection;
	}

	public static List<ApDevice> getApsAvailable() {
		return apsAvailable;
	}

	public static void setApsAvailable(List<ApDevice> apsAvailable) {
		Migration.apsAvailable = apsAvailable;
	}

	public static List<FogDevice> getServerCloudletsAvailable() {
		return serverCloudletsAvailable;
	}

	public static void setServerCloudletsAvailable(List<FogDevice> serverCloudletsAvailable) {
		Migration.serverCloudletsAvailable = serverCloudletsAvailable;
	}

	public static int getPolicyReplicaVM() {
		return policyReplicaVM;
	}

	public static void setPolicyReplicaVM(int policyReplicaVM) {
		Migration.policyReplicaVM = policyReplicaVM;
	}




	/**
	 * 
	 * @param distance 
	 * @return boolean about if is or isn't in migration point 
	 */





}
