#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>


/**
 * \file        Python wrapper for IK/FK in kinematics::KinematicsBase
 * \author      Nicolas Alt
 * \date        2014-08-26
 */

namespace py = boost::python;

// http://stackoverflow.com/questions/3761391/boostpython-python-list-to-stdvector
template< typename T > inline std::vector< T > to_std_vector( const py::object& iterable ) {
    return std::vector< T >( py::stl_input_iterator< T >( iterable ),
                             py::stl_input_iterator< T >( ) );
}

// http://stackoverflow.com/questions/6157409/stdvector-to-boostpythonlist
template<typename T> py::list to_py_list(const std::vector<T>& v) {
	py::list l;
	typename std::vector<T>::const_iterator it;
	for (it = v.begin(); it != v.end(); ++it)
	    l.append(*it);   
	return l;  
}

// The plugin file defines PLUGINLIB_EXPORT_CLASS(ikfast_kinematics_plugin::IKFastKinematicsPlugin, ...
// lmtclam_moveit_ikfast/package.xml exports the plugin in the name of moveit_core
pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");

/** \brief Helper class for methods in kinematics::KinematicsBase. These methods are exported to Python. */
class kinematics_wrapper
{
    private:
        boost::shared_ptr<kinematics::KinematicsBase> ik;

	public:
		kinematics_wrapper(std::string node_name)
        {
			int c = 0;
			ros::init(c, NULL, node_name);

            try {
                ik = loader.createInstance("lmtclam_arm_kinematics/IKFastKinematicsPlugin");  
            } catch(pluginlib::PluginlibException& ex) {
                ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
            }
            printf("ik:%x\n", (void*)ik.get());
			//ik = new ikfast_kinematics_plugin::IKFastKinematicsPlugin();
		}
		~kinematics_wrapper()  {  }
		std::string greet() { return "Hello world"; }
		std::string getBaseFrame() { return ik->getBaseFrame(); }
		bool initialize(std::string robot_description, std::string group_name, std::string base_name, std::string tip_name, double search_discretization) { 
            if (!ik) {
                ROS_ERROR("KinematicsBase object is NULL");
                return false;
            }	
			return ik->initialize(robot_description, group_name, base_name, tip_name, search_discretization);
        }

		py::object getPositionFK(std::string link_name, py::list joint_angles)
		{
			std::vector< geometry_msgs::Pose > _poses;
			std::vector < double > _joint_angles = to_std_vector<double>(joint_angles);
			std::vector < std::string > _link_names; _link_names.push_back(link_name);

			bool r = ik->getPositionFK(_link_names, _joint_angles, _poses);

			if (r && _poses.size() >= 1) {
				py::dict pose;
				py::list pose_t, pose_q;
				pose_t.append<double>(_poses[0].position.x);
				pose_t.append<double>(_poses[0].position.y);
				pose_t.append<double>(_poses[0].position.z);
				pose_q.append<double>(_poses[0].orientation.w);
				pose_q.append<double>(_poses[0].orientation.x);
				pose_q.append<double>(_poses[0].orientation.y);
				pose_q.append<double>(_poses[0].orientation.z);

				pose["position"] = pose_t;
				pose["orientation"] = pose_q;
				return pose;
			} else
				return py::object();
		}

		py::object searchPositionIK(py::list ik_position, py::list ik_orientation, py::list ik_seed_state, double timeout)
		{
			geometry_msgs::Pose _ik_pose;
			_ik_pose.position.x = py::extract<double>(ik_position[0]);
			_ik_pose.position.y = py::extract<double>(ik_position[1]);
			_ik_pose.position.z = py::extract<double>(ik_position[2]);
			_ik_pose.orientation.w = py::extract<double>(ik_orientation[0]);
			_ik_pose.orientation.x = py::extract<double>(ik_orientation[1]);
			_ik_pose.orientation.y = py::extract<double>(ik_orientation[2]);
			_ik_pose.orientation.z = py::extract<double>(ik_orientation[3]);
			std::vector< double > _ik_seed_state = to_std_vector<double>(ik_seed_state);
			std::vector< double > _consistency_limits, _solution;
			moveit_msgs::MoveItErrorCodes _error_code;
			bool r = ik->searchPositionIK (_ik_pose, _ik_seed_state, timeout, _consistency_limits, _solution, _error_code);
			if (r)
				return to_py_list<double>(_solution);
			else
				return py::object();
		}
};

// typedef kinematics::KinematicsBase KinematicsType;
// typedef ikfast_kinematics_plugin::IKFastKinematicsPlugin KinematicsType;
// typedef test_class KinematicsType;


/*struct kb_callback : KinematicsType {
    kb_callback(PyObject *p) : self(p) {}
    //int pure(int x) { return call_method<int>(self, "pure", x); }
    PyObject *self;
};*/



typedef kinematics_wrapper KinematicsType;

/** \brief Defiens the Python module */
BOOST_PYTHON_MODULE(kinematics_py)
{
    // Create the Python type object for our extension class and define __init__ function.
    //class_<KinematicsType, boost::noncopyable, boost::shared_ptr<kb_callback> >("kinematics_base")
    //class_<kinematics::KinematicsBase  , boost::noncopyable>("kinematics_base", no_init)
    //class_<KinematicsType, boost::noncopyable, boost::shared_ptr<kb_callback> >("kinematics_base")
    py::class_<KinematicsType>("Kinematics", py::init<std::string>())
    	.def("greet", &KinematicsType::greet)
    	.def("initialize", &KinematicsType::initialize)
        .def("getBaseFrame", &KinematicsType::getBaseFrame)
        .def("getPositionFK", &KinematicsType::getPositionFK)
        .def("searchPositionIK", &KinematicsType::searchPositionIK)
    ;
}