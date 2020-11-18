//#include <cstdlib>
#include <sstream>
#include <iostream>
#include <vector>


using namespace std;
//timestamp tx ty tz qx qy qz qw
struct Quaternionf{

	double x;
	double y;
	double z;
	double w;

};

struct Vector3f{
	double qx;
	double qy;
	double qz;
};

struct TumPose
{
  double timestamp;
  struct Quaternionf rot;
  struct Vector3f trs;
};


inline std::istream& operator>>(std::istream& is, TumPose& pose)
{
  is >> std::setprecision(16) >> pose.timestamp >> pose.trs(0) >> pose.trs(1) >> pose.trs(2)
     >> pose.rot.x() >> pose.rot.y() >> pose.rot.z() >> pose.rot.w();
  return is;
}

inline std::ostream& operator<<(std::ostream& is, TumPose& pose)
{
  is << std::setprecision(16) << std::to_string(pose.timestamp) << " " << pose.trs(0) << " " << pose.trs(1) << " "
     << pose.trs(2) << " " << pose.rot.x() << " " << pose.rot.y() << " "
     << pose.rot.z() << " " << pose.rot.w();
  return is;
}


std::vector<TumPose> LoadPoses(std::string data_path)
{
  std::string file_path = data_path + "/groundtruth.txt";
  std::ifstream f(file_path);

  if (!f.is_open())
  {
    std::cout << "groundtruth.txt not present, not loading the poses" <<std::endl;
    return {};
  }

  std::vector<TumPose> poses;
  TumPose pose;
  std::string line;
  while (std::getline(f, line)) {
    if (line.find("#") != std::string::npos) // skip comments
      continue;

    std::stringstream ss(line);
    ss >> pose;
    //pose.rot.normalize();
    poses.push_back(pose);
  }
  return poses;
}

int main(int argc, char **argv )
{
   LoadPoses("/root/EVA4P2/SkunkWorks/rgbd_dataset_freiburg1_desk")
}
