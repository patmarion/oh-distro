#ifndef RENDERER_CRAWLINGPLAN_ROBOTPLANLISTENER_HPP
#define RENDERER_CRAWLINGPLAN_ROBOTPLANLISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/file_access_utils.hpp>
#include <sys/time.h>
#include <time.h>

namespace renderer_crawling_plan 
{

  class RobotPlanListener
  {
    //--------fields
    
   public:
    std::string _robot_name;
    
   private:      
    std::string _urdf_xml_string;   
   
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false

    boost::shared_ptr<lcm::LCM> _lcm;    
    
    //get rid of this
    BotViewer *_viewer;

    
    bool _urdf_parsed;
    bool _urdf_subscription_on;
    bool _aprvd_walking_goal_in_cache;
    bool _is_multi_approve_plan;
    bool _plan_paused; // a manip plan is paused on pressing the pause button, unpaused on pressing play again.
    
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_robot;
    //boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _base_gl_robot;
    //----------------constructor/destructor
    
    int _in_motion_keyframe_index;


  public:
    bool _is_keyframe_plan;
    int64_t _last_plan_msg_timestamp; 
    RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer, int operation_mode);
    ~RobotPlanListener();
    
   // boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    //std::vector< boost::shared_ptr<visualization_utils::GlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_keyframe_list;
    std::vector< int64_t >  _keyframe_timestamps;
    std::vector< int >  _breakpoint_indices;
    int _active_breakpoint;
    int _num_breakpoints;
    
    //The following are local in-motion copies that appear on doubleclk of a keyframe and can be moved around via markers
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_left_hand;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_right_hand;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_left_foot;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_right_foot;
    //-------------message callback
    
    drc::robot_plan_t _received_plan;
    drc::walking_goal_t _received_walking_goal;
    
    int64_t _controller_utime;
    int8_t _controller_status;
  

    void commit_walking_goal(int64_t utime,std::string &channel);  
    void commit_plan_control(int64_t utime, std::string &channel,bool pause, bool terminate);

    void set_in_motion_hands_state(int index)
    {
 
      KDL::Frame T_base_palm,T_world_palm_l,T_world_palm_r,T_world_base_l,T_world_base_r;
     
      T_base_palm = KDL::Frame::Identity();
      T_base_palm.M = KDL::Rotation::RPY(0,-M_PI/2,0);
      _gl_robot_keyframe_list[index]->get_link_frame("left_palm",T_world_palm_l);
      _gl_robot_keyframe_list[index]->get_link_frame("right_palm",T_world_palm_r);

      T_world_base_l = T_world_palm_l*(T_base_palm.Inverse());
      T_world_base_r = T_world_palm_r*(T_base_palm.Inverse());
      

      
      // Flip marker direction to always point away from the body center.

      double normal, flipped;
      Eigen::Vector3f u_x(1,0,0);
      Eigen::Vector3f u_y(0,1,0);
      Eigen::Vector3f u_hand_to_body;
      u_hand_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_palm_l.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_palm_l.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_palm_l.p[2]; 
      u_hand_to_body.normalize();
      
      normal = acos(u_hand_to_body.dot(u_x));
      flipped = acos(u_hand_to_body.dot(-u_x));
      if(flipped>normal+1e-1) {
        _gl_left_hand->flip_trans_marker_xdir(true);
        }
      else{
       _gl_left_hand->flip_trans_marker_xdir(false);
       }
       
      normal = acos(u_hand_to_body.dot(u_y));
      flipped = acos(u_hand_to_body.dot(-u_y));
      if(flipped>normal+1e-1){
        _gl_left_hand->flip_trans_marker_ydir(true);
        }
      else{
       _gl_left_hand->flip_trans_marker_ydir(false); 
       }
      u_hand_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_palm_r.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_palm_r.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_palm_r.p[2]; 
      u_hand_to_body.normalize();
      
      normal = acos(u_hand_to_body.dot(u_x));
      flipped = acos(u_hand_to_body.dot(-u_x));
      if(flipped>normal+1e-1){
        _gl_right_hand->flip_trans_marker_xdir(true);
        }
      else{
       _gl_right_hand->flip_trans_marker_xdir(false);
       }
      normal = acos(u_hand_to_body.dot(u_y));
      flipped = acos(u_hand_to_body.dot(-u_y));
      if(flipped>normal+1e-1){
        _gl_right_hand->flip_trans_marker_ydir(true);
        }
      else{
       _gl_right_hand->flip_trans_marker_ydir(false);  
       }
       
      std::map<std::string, double> jointpos_l;
      jointpos_l=_gl_left_hand->_current_jointpos;
      _gl_left_hand->set_state(T_world_base_l,jointpos_l);
      _gl_left_hand->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
       std::map<std::string, double> jointpos_r;
      jointpos_r = _gl_right_hand->_current_jointpos;
      _gl_right_hand->set_state(T_world_base_r,jointpos_r);    
      _gl_right_hand->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
      _in_motion_keyframe_index = index;
    
    };
    
    void set_in_motion_feet_state(int index)
    {
 
      KDL::Frame T_world_foot_l,T_world_foot_r;
     
      _gl_robot_keyframe_list[index]->get_link_frame("l_foot",T_world_foot_l);
      _gl_robot_keyframe_list[index]->get_link_frame("r_foot",T_world_foot_r);

      // Flip marker direction to always point away from the body center.

      double normal, flipped;
      Eigen::Vector3f u_x(1,0,0);
      Eigen::Vector3f u_y(0,1,0);
      Eigen::Vector3f u_foot_to_body;
      u_foot_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_foot_l.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_foot_l.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_foot_l.p[2]; 
      u_foot_to_body.normalize();
      
      normal = acos(u_foot_to_body.dot(u_x));
      flipped = acos(u_foot_to_body.dot(-u_x));
      if(flipped>normal+1e-1) 
      {
        _gl_left_foot->flip_trans_marker_xdir(true);
      }
      else
      {
        _gl_left_foot->flip_trans_marker_xdir(false);
      }
       
      normal = acos(u_foot_to_body.dot(u_y));
      flipped = acos(u_foot_to_body.dot(-u_y));
      if(flipped>normal+1e-1)
      {
        _gl_left_foot->flip_trans_marker_ydir(true);
      }
      else
      {
        _gl_left_foot->flip_trans_marker_ydir(false); 
      }
      u_foot_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_foot_r.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_foot_r.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_foot_r.p[2]; 
      u_foot_to_body.normalize();
      
      normal = acos(u_foot_to_body.dot(u_x));
      flipped = acos(u_foot_to_body.dot(-u_x));
      if(flipped>normal+1e-1){
        _gl_right_foot->flip_trans_marker_xdir(true);
        }
      else{
       _gl_right_foot->flip_trans_marker_xdir(false);
       }
      normal = acos(u_foot_to_body.dot(u_y));
      flipped = acos(u_foot_to_body.dot(-u_y));
      if(flipped>normal+1e-1)
      {
        _gl_right_foot->flip_trans_marker_ydir(true);
      }
      else
      {
        _gl_right_foot->flip_trans_marker_ydir(false);  
      }
       
      std::map<std::string, double> jointpos_l;
      jointpos_l=_gl_left_foot->_current_jointpos;
      _gl_left_foot->set_state(T_world_foot_l,jointpos_l);
      _gl_left_foot->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
       std::map<std::string, double> jointpos_r;
      jointpos_r = _gl_right_foot->_current_jointpos;
      _gl_right_foot->set_state(T_world_foot_r,jointpos_r);    
      _gl_right_foot->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
      _in_motion_keyframe_index = index;
    
    };    
    
    bool is_in_motion(int index) {  
        return (index==_in_motion_keyframe_index);
    };
    
    
    bool is_keyframe_plan()
    {
     return _is_keyframe_plan;
    };
		
		bool is_plan_paused()
		{
		  return _plan_paused;
		};
		
		bool is_multi_approval_plan()
    {
		    return _is_multi_approve_plan;
		};
		
		void purge_current_plan(){
		    _gl_robot_list.clear();
        if(_is_keyframe_plan){
        _gl_robot_keyframe_list.clear();
        _gl_left_hand->enable_bodypose_adjustment(false); 
        _gl_right_hand->enable_bodypose_adjustment(false);
        _gl_left_foot->enable_bodypose_adjustment(false); 
        _gl_right_foot->enable_bodypose_adjustment(false);
        _is_keyframe_plan = false;
        }

		};	
		void activate_crawling_plan()
    {
		  _is_keyframe_plan =false;
      deactivate_multi_approval();  
		};
		bool activate_keyframe_plan()
    {
      _is_keyframe_plan =true;
      if(_aprvd_walking_goal_in_cache){
       _aprvd_walking_goal_in_cache=false;
      }
      deactivate_multi_approval();
		};
		
		bool activate_multi_approval()
    {
      _is_multi_approve_plan =true;
      _active_breakpoint=0;
		};
		
		bool deactivate_multi_approval()
    {
      _is_multi_approve_plan =false;
      _active_breakpoint=0;
		};
    
    
    int64_t get_keyframe_timestamp(int index) {
        return _keyframe_timestamps[index];
    };
    
  private:
   void handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_plan_t* msg);	
   void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);
   void handleAprvWalkingGoalMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
						 const drc::walking_goal_t* msg); 
   void handleControllerStatusMsg(const lcm::ReceiveBuffer* rbuf,
                                                 const string& chan, 
                                                 const drc::controller_status_t* msg);
	 bool load_hand_urdfs(std::string &_left_hand_urdf_xml_string,std::string &_right_hand_urdf_xml_string);
   bool load_foot_urdfs(std::string &_left_foot_urdf_xml_string,std::string &_right_foot_urdf_xml_string);

}; //class RobotPlanListener
  

} //namespace renderer_crawling_plan


#endif //RENDERER_CRAWLINGPLAN_ROBOTPLANLISTENER_HPP