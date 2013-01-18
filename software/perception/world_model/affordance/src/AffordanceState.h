/*
 * AffordanceState.h
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCESTATE_H_
#define AFFORDANCESTATE_H_

#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <boost/tuple/tuple.hpp>

namespace affordance
{


  class ArgumentException : public std::runtime_error {public: ArgumentException(const std::string &msg) : std::runtime_error(msg){}};
  class InvalidOtdfID : public std::runtime_error { public: InvalidOtdfID(const std::string &msg) : std::runtime_error(msg){}};
  class KeyNotFoundException : public std::runtime_error { public: KeyNotFoundException(const std::string &msg) : std::runtime_error(msg){}};

  typedef std::pair<const int32_t, const int32_t> GlobalUID;
  
  class AffordanceState
  {
    //------------fields
  public:
    enum OTDF_TYPE  {CYLINDER 	= drc::affordance_t::CYLINDER,
		     LEVER 	 	= drc::affordance_t::LEVER,
		     SPHERE		= drc::affordance_t::SPHERE,
		     BOX 		= drc::affordance_t::BOX,
		     UNKNOWN};
    
    /**standardizing the naming for common fields in drc::affordance_t. 
       These should be used as keys in the _params map*/
    static std::string  X_NAME, Y_NAME, Z_NAME, ROLL_NAME, PITCH_NAME, YAW_NAME,
      RADIUS_NAME, LENGTH_NAME, WIDTH_NAME, HEIGHT_NAME,  R_COLOR_NAME, G_COLOR_NAME, B_COLOR_NAME;
    
  private:
    boost::unordered_map<int16_t, OTDF_TYPE> idToEnum;
    
  public: //should make get / private set methods for these
    //mimicking lcm
    int64_t    _map_utime;
    int32_t    _map_id;
    
    /**which object in the scene?*/
    int32_t    _object_id;
    
    /**type of object*/
    OTDF_TYPE  _otdf_id;
    
    /**informal name for the affordance*/
    std::string _name;
    
    /**{name --> value} maps*/
    boost::unordered_map<std::string, double> _params, //geometrical properties
                                             _states;
    std::vector< int32_t > _ptinds;
    
    //-----------constructor/destructor
  public:
    AffordanceState(const drc::affordance_t *affordanceMsg);
    AffordanceState(const std::string &name,
		    const int &objId = 0, const int &mapId = 0,
		    const KDL::Frame &frame = KDL::Frame(KDL::Vector(0,0,0)),
		    const Eigen::Vector3f &color = Eigen::Vector3f(1,0,0));
    AffordanceState(const AffordanceState &other);
    AffordanceState& operator=( const AffordanceState& rhs );
    
    void initHelper(const drc::affordance_t *msg);
    void initIdEnumMap();
    virtual ~AffordanceState();
    
    //observers
  public:
    void toMsg(drc::affordance_t *affordanceMsg) const;
    GlobalUID getGlobalUniqueId() const;
    
    std::string getName() const;
    
    //--these methods throw exceptions if we don't
    //have these fields defined
    Eigen::Vector3f getXYZ() const;
    Eigen::Vector3f getRPY() const;
    void getFrame(KDL::Frame &frame) const;
    Eigen::Vector3f getColor() const;
    
    bool hasRPY() const;
    double radius() const;
    double length() const;
    double width() const;
    double height() const;
    
    
    //helpers
  private:
    static void assertContainsKey(const boost::unordered_map<std::string, double> &map,
				  const std::string &key);
  public:
    static std::string toStr(boost::unordered_map<std::string,double> m);
    static std::string toStr(double d);
    
  };
  
  std::ostream& operator<<( std::ostream& out, const AffordanceState& other );
  
  typedef boost::shared_ptr<AffordanceState> AffPtr;
  
} //namespace affordance

#endif /* AFFORDANCESTATE_H_ */
