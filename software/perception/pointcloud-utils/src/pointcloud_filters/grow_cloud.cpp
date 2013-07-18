#include "grow_cloud.hpp"

using namespace std;
using namespace pcl;
using namespace pcl::octree;



GrowCloud::GrowCloud () {
  verbose_lcm =0;
  verbose_text =0;
  pose_element_id =-1;
  pose_coll_id =-1;
}
 
void GrowCloud::doGrowCloud (vector<BasicPlane> &outstack) {

  float resolution = 128.0f;
  
/*  Ptcoll_cfg ptcoll_cfg;
  ptcoll_cfg.point_lists_id =0; //bot_timestamp_now();
  ptcoll_cfg.collection = 0;
  ptcoll_cfg.element_id = 0;*/
  
  pcl::PointIndices::Ptr searchstack (new pcl::PointIndices);
  int n_found_clouds=0;
  int step;
  float search_radius = 0.2; // 40cm
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  while(incloud->size() >=0){
/*    ptcoll_cfg.id = 2;
    ptcoll_cfg.reset=true;
    ptcoll_cfg.name ="Remaining Cloud";
    ptcoll_cfg.npoints =	 incloud->points.size();
    ptcoll_cfg.rgba ={0.0,0.0,1,0};
    ptcoll_cfg.type=1;
    pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *incloud);
    
    ptcoll_cfg.id = 3;
    ptcoll_cfg.reset=true;
    ptcoll_cfg.name ="Out Cloud";
    ptcoll_cfg.npoints =	 outcloud->points.size();
    ptcoll_cfg.rgba ={0.0,1,1,0};
    ptcoll_cfg.type=1;
    pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *outcloud);  
    cout << incloud->size() << " incloud | " << searchstack->indices.size() << " searchstack \n";
*/

    // if searchstack is not empty
    if (searchstack->indices.size() >0){
      //cout <<  "incloud size " <<  searchstack->indices.size() << " - not empty\n";
      // 11. select a point from the searchstack, find all neighbours in the incloud within r
      pcl::PointXYZRGB searchPoint;
      searchPoint = outcloud->points[searchstack->indices[0]];
      pcl::octree::OctreePointCloudSearch<PointXYZRGB> octree (resolution);
      octree.setInputCloud (incloud);
      octree.addPointsFromInputCloud ();      
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      int pts_nearby = octree.radiusSearch (searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      //std::cerr << pts_nearby << " neighbors within radius search at (" << searchPoint.x
      //	<< " " << searchPoint.y << " " << searchPoint.z  << ") with radius=" << search_radius << std::endl;
      
      // 12. put these neighbours into the outcloud stack and the searchstack, remove them from incloud
      if ( pts_nearby > 0) {
	std::sort( pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end() ); // sort the incoming files:
	for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
	  //std::cerr <<  pointIdxRadiusSearch[i] << ":    "  <<   cloud_filtered->points[ pointIdxRadiusSearch[i] ].x
	  //    << " " << cloud_filtered->points[ pointIdxRadiusSearch[i] ].y
	  //    << " " << cloud_filtered->points[ pointIdxRadiusSearch[i] ].z
	  //    << " (distance: " << sqrt(pointRadiusSquaredDistance[i]) << ")" << std::endl;
	  outcloud->points.push_back(incloud->points[pointIdxRadiusSearch[i]]);
	  searchstack->indices.push_back(outcloud->points.size()-1);
	}
	for (int i = pointIdxRadiusSearch.size ()-1; i>=0; i--){
	  incloud->points.erase (incloud->points.begin()+ pointIdxRadiusSearch[i]);
	}
      
	// 13. remove point from search stack
	searchstack->indices.erase (searchstack->indices.begin());
      }else{
      //cout << "nothing nearby - toss this point"; 
      // cin >> step;
      searchstack->indices.erase (searchstack->indices.begin());
      }
      
      //cout << "to continue enter int ";
      //cin >> step;
    }else{
      if( outcloud->points.size() > min_cloud_size_){ // minimum size of the cloud
/*	ptcoll_cfg.id = 4+n_found_clouds;
	ptcoll_cfg.reset=false;
	ptcoll_cfg.name ="Found Cloud";
	ptcoll_cfg.npoints =	 outcloud->points.size();
	ptcoll_cfg.rgba ={0.0,1   ,0.0,0};
	ptcoll_cfg.type=1;
	pcdXYZRGB_to_lcm(publish_lcm,ptcoll_cfg, *outcloud);  
*/

	BasicPlane one_plane;
	// not currently filled in:
	//one_plane.utime
	//one_plane.major
	//one_plane.minor
	compute3DCentroid (*outcloud,one_plane.centroid);
	one_plane.cloud = (*outcloud);
	outstack.push_back(one_plane);
	
	n_found_clouds++;
	//cout << "found this cloud of size, " << outcloud->points.size() << " \n";
	//cin >> step;
      }else{
	
      }
      if (incloud->points.empty())
	break;
      
      outcloud->points.clear();
      searchstack->indices.clear();
      outcloud->points.push_back(incloud->points[0]);
      searchstack->indices.push_back(0);
      incloud->points.erase (incloud->points.begin());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      if (incloud->points.empty())
	break;
      
    }
  }  
}
  
  
  
void GrowCloud::printPlaneStackCoeffs (vector<BasicPlane> &planeStack,  std::stringstream &ss) {
  for (size_t i=0; i < planeStack.size() ;  i++){
    BasicPlane p = planeStack[i];
    ss <<  p.coeffs.values[0] << ","
       <<  p.coeffs.values[1] << ","
       <<  p.coeffs.values[2] << ","       
       <<  p.coeffs.values[3] << "\n";
  }  
}

void GrowCloud::printPlaneStackHull (vector<BasicPlane> &planeStack,  std::stringstream &ss) {
  for (size_t i=0; i < planeStack.size() ;  i++){
    for (size_t j=0; j <planeStack[i].cloud.points.size(); j++){
      pcl::PointXYZRGB pt = planeStack[i].cloud.points[j];
      ss << pt.x << "," <<  pt.y << "," <<  pt.z ;
      if (j != planeStack[i].cloud.points.size() -1){
        ss << ","       ;
      }else{
        ss << "\n";
      }
    }
  }  
}




void GrowCloud::visualizePlanes(vector<BasicPlane> &plane_stack,pointcloud_vis* pc_vis, int null_id  ){
  obj_cfg ocfg = obj_cfg(null_id,"Plane Detect | Null Pose",5,0);
  Isometry3dTime null_poseT_ = Isometry3dTime(0, Eigen::Isometry3d::Identity() );
  pc_vis->pose_to_lcm(ocfg, null_poseT_);

  float colors_a[] ={0.3,0.8,0.1};
  std::vector<float> colors_a_v;
  colors_a_v.assign(colors_a,colors_a+4*sizeof(float));
  float colors_b[] ={0.3,0.8,0.1};
  std::vector<float> colors_b_v;
  colors_b_v.assign(colors_b,colors_b+4*sizeof(float));
  
  std::vector < pcl::PointCloud<pcl::PointXYZRGB> > clouds;
  for (size_t i = 0; i< plane_stack.size() ; i++){
    clouds.push_back(  plane_stack[i].cloud  );
  } 
  ptcld_cfg pcfg = ptcld_cfg(null_id+1,    "Plane Detect | Extracted Planes"  ,3,1, null_id,1,colors_a_v );
  pc_vis->ptcld_collection_to_lcm(pcfg, clouds, null_poseT_.utime, null_poseT_.utime );

  std::vector < pcl::PointCloud<pcl::PointXYZRGB> > normal_clouds;
  for (size_t i=0;i<plane_stack.size();i++){
    BasicPlane plane = plane_stack[i];
    if (plane.cloud.points.size() ==0 ){
      cout <<"ERROR: CONVEX HULL HAS NO POINTS! - NEED TO RESOLVE THIS\n"; 
      continue;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    normal_cloud->points.push_back( plane.cloud.points[0]);
    pcl::PointXYZRGB pt;
    pt.x= plane.cloud.points[0].x + 0.2*plane.coeffs.values[0];
    pt.y= plane.cloud.points[0].y + 0.2*plane.coeffs.values[1];
    pt.z= plane.cloud.points[0].z + 0.2*plane.coeffs.values[2];
    pt.r =0;      pt.g =255;      pt.b =0;
    normal_cloud->points.push_back( pt );
    normal_clouds.push_back( *normal_cloud );
  } 
  ptcld_cfg pcfg2 = ptcld_cfg(null_id +2,    "Plane Detect | Extracted Normals"    ,3,1, null_id,1, colors_b_v  );
  pc_vis->ptcld_collection_to_lcm(pcfg2, normal_clouds, null_poseT_.utime, null_poseT_.utime );        
  
  
}