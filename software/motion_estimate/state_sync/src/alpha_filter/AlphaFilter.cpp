#include "AlphaFilter.hpp"

AlphaFilter::AlphaFilter(const std::set<std::string> &target_joints, float alpha)
    : _configured(false), _alpha(alpha), _n_alpha(1-alpha), _target_joints(target_joints)
{
    if( !(0.0<alpha && alpha<=1.0)) {
        std::cerr<<"Alpha is out of range! Truncating to [0,1]."<<std::endl;
        // truncate alpha value to valid range
        alpha = (alpha < 0.0) ? 0.0 : alpha;
        alpha = (alpha > 1.0) ? 1.0 : alpha;
    }
}

void AlphaFilter::setup(const std::vector<std::string> &name, const std::vector<float> &values) {
    // reset
    _target_index.clear();
    _values.clear();

    // find target joints by name and store index unless all target joints are processed
    for(unsigned int i = 0; i<name.size() && _target_index.size() != _target_joints.size(); i++) {
        if(_target_joints.count(name[i])) {
            _target_index.push_back(i);
            _values.push_back(values[i]);
        }
    }

    assert(_target_index.size() == _target_joints.size());
    assert(_values.size() == _target_joints.size());

    _configured = true;
}

void AlphaFilter::update(std::vector<float> &measurement) {
    for(unsigned int i = 0; i<_values.size(); i++) {
        _values[i] = _alpha * measurement[_target_index[i]] +  _n_alpha * _values[i];
        measurement[_target_index[i]] = _values[i];
    }
}
