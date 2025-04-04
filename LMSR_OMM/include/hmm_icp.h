#ifndef _HMM_ICP_H_
#define _HMM_ICP_H_
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>
#include <cmath>
#include <ctime>
#include <algorithm> 
#include <float.h>
#include <sstream>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/point_cloud_color_handlers.h>
// #include <boost/thread/thread.hpp>
namespace flann{
template <typename T> struct Dist_Label;

template<class T>
struct Dist_Label
{
    Dist_Label(){}
    Dist_Label(double type_factor):type_factor_(type_factor){}
    double type_factor_=3;
    typedef bool is_kdtree_distance;

    typedef T ElementType;
    typedef typename Accumulator<T>::Type ResultType;

    /**
     *  Compute the squared Euclidean distance between two vectors.
     *
     *	This is highly optimised, with loop unrolling, as it is one
     *	of the most expensive inner loops.
     *
     *	The computation of squared root at the end is omitted for
     *	efficiency.
     */
    template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
    {
        ResultType result = ResultType();
        ResultType diff0, diff1, diff2, diff3;
        Iterator1 last = a + size;
        Iterator1 lastgroup = last - 3;

        /* Process 4 items with each loop for efficiency. */
        while (a < lastgroup) {
            diff0 = (ResultType)(a[0] - b[0]);
            diff1 = (ResultType)(a[1] - b[1]);
            diff2 = (ResultType)(a[2] - b[2]);
            // diff3 = (ResultType)(a[3] - b[3]);
            diff3 = a[3] == b[3]?0:type_factor_;
            result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
            a += 4;
            b += 4;

            if ((worst_dist>0)&&(result>worst_dist)) {
                return result;
            }
        }
        /* Process last 0-3 pixels.  Not needed for standard vector lengths. */
        while (a < last) {
            diff0 = (ResultType)(*a++ - *b++);
            result += diff0 * diff0;
        }
        return result;
    }

    /**
     *	Partial euclidean distance, using just one dimension. This is used by the
     *	kd-tree when computing partial distances while traversing the tree.
     *
     *	Squared root is omitted for efficiency.
     */
    template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int) const
    {
        return (a-b)*(a-b);
    }
};
}
template <typename PointT, typename Dist = flann::Dist_Label<float>>
class myKdTreeFLANN : public pcl::KdTreeFLANN<PointT,Dist> 
{
public:
    // int dim_;
    // void setDim(int dim)
    // {
    //     dim_=dim;
    // }

};
class icp_4d{
  //member-----------------------------------
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target_cloud_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source_cloud_;
  
  //kdtree *tree;
  myKdTreeFLANN<pcl::PointXYZL> kdtree_; //Not fully functional

  std::vector< std::pair<int, int> > correspond_;
  int match_num_;
  Eigen::Matrix4f transform_guess_;
  Eigen::Vector3f translate_guess_;
  Eigen::Matrix3f rotation_guess_;
  Eigen::Matrix4f transform_result_;


  //convergence criteria
  //1) error value -> 0
  //2) number of associated new points -> 0
  //3) association stability measure -> 0
  double error_value_;
  int num_of_new_point_;
  int stability_;
  //---------------------------------//member

 public:
  //setter
  int getMatchNum(){return match_num_;}
  void setTarget(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target_cloud);
  void setSource(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source_cloud);

  //getter
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> getTarget();
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> getSource();

  std::vector< std::pair<int, int> > getCorrespond();
  void setTransformation(Eigen::Matrix4f init_transform);
  Eigen::Matrix4f getTransformation();

  double getEpsilon();
  int getNumberOfNewCorrespond();

  void setMaxRange(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> output,
				   std::shared_ptr<const pcl::PointCloud<pcl::PointXYZL>>  source, 
				   double max_range);
  
  void normalizePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> output,
						   std::shared_ptr<const pcl::PointCloud<pcl::PointXYZL>>  source,
						   double max_range,
						   double max_intensity);

  void estimateCorrespond(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target, 
						  std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source,
						  double threshold_dst,double heading,double ICP_MATCH_DIS);

  bool estimateCorrespond_4d(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target, 
							 std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source,
							 double heading,
              double type_factor,
              double threshold_dst,
              int ICP_MIN_POINTS,
              double ICP_MATCH_DIS);
  
  double estimateEpsilon(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target, 
						 std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source,double heading,double ICP_MATCH_DIS);

  int estimateStability(std::vector< std::pair<int, int> > new_correspond,
						std::vector< std::pair<int, int> > old_correspond);

  void estimateTransform(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target, 
						 std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source);

  void applyFastICP(std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> target, 
					std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> source);

  void applyICP(double max_range,
  				double max_intensity,
  				double threshold_radius,
				Eigen::Matrix4f init_guess,double heading,double ICP_MATCH_DIS);

};





#endif