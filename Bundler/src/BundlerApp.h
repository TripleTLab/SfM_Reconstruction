#ifndef __bundlerapp_h__
#define __bundlerapp_h__

#include "BaseApp.h"
#include "LinkDirection.h"
#include "TwoFrameModel.h"

typedef std::pair<int,int> ImagePair;

class BundlerApp : public BaseApp
{
public:
    BundlerApp() {
        /* 设置初始参数 */
        m_bundle_version = 0.3;

        m_fisheye = false;
        m_fixed_focal_length = true;
        m_estimate_distortion = false;
        m_construct_max_connectivity = false;
        m_bundle_provided = false;
        m_analyze_matches = false;

        m_estimate_ignored = false;

        m_use_constraints = false;
        m_constrain_focal = false;
        m_constrain_focal_weight = 100.0;
        m_distortion_weight = 1.0e2;

        m_use_point_constraints = false;
        m_point_constraint_weight = 0.0;
        m_point_constraints = NULL;
        m_point_constraint_file = NULL;

        m_only_bundle_init_focal = false;
        m_init_focal_length = 532.0;
        m_initial_pair[0] = -1;
        m_initial_pair[1] = -1;

        m_panorama_mode = false;
        m_homography_threshold = 6.0;
        m_homography_rounds = 256;
        m_fmatrix_threshold = 9.0;
        m_fmatrix_rounds = 2048;
        m_skip_fmatrix = false;
        m_skip_homographies = false;
        m_projection_estimation_threshold = 4.0; // 1.8;
        m_min_proj_error_threshold = 8.0;
        m_max_proj_error_threshold = 16.0;
        m_min_camera_distance_ratio = 0.0;
        m_baseline_threshold = -1.0;
        m_optimize_for_fisheye = false;
        m_use_focal_estimate = false;
        m_trust_focal_estimate = false;
        m_factor_essential = true;
        m_up_image = -1;
        // m_start_camera = -1;
        m_min_track_views = 2;
        m_max_track_views = 100000;
        m_min_num_feat_matches = 16;
        m_min_max_matches = 16;
        m_num_matches_add_camera = -1; /* No maximum by default */
        m_ray_angle_threshold = 2.0;

        m_keypoint_border_width = 0;
        m_keypoint_border_bottom = 0;

        m_fisheye_params = NULL;
        m_bundle_output_file = m_bundle_output_base = NULL;
        m_bundle_file = NULL;
        m_intrinsics_file = NULL;
        m_match_directory = ".";
        m_match_index_dir = NULL;
        m_match_table = NULL;
        m_key_directory = ".";
        m_image_directory = ".";
        m_output_directory = ".";
        m_use_intrinsics = false;

        m_matches_computed = false;
        m_ann_max_pts_visit = 400;

        m_matches_loaded = false;
        m_features_coalesced = false;

        m_assemble = false;
        m_run_bundle = false;
        m_rerun_bundle = false;
        m_fast_bundle = true;
#ifdef __USE_CERES__
        m_use_ceres = false;
#endif /* __USE_CERES__ */
        m_skip_full_bundle = false;
        m_skip_add_points = false;
        m_use_angular_score = false;

        m_compress_list = false;
        m_reposition_scene = false;
        m_prune_bad_points = false;
        m_predict_next_image = false;
        m_prediction_image = NULL;
        m_scale_focal = 1.0;
        m_scale_focal_file = NULL;
        m_rotate_cameras_file = NULL;
        m_output_relposes = false;
        m_output_relposes_file = NULL;

        m_compute_covariance = false;
        m_covariance_fix1 = -1;
        m_covariance_fix2 = -1;

        m_track_file = NULL;
        m_zero_distortion_params = false;
        m_enrich_points = false;
        m_fix_necker = false;



        m_ignore_file = NULL;
        m_add_image_file = NULL;
        m_add_images_fast = false;

        m_scale = 1.0;
        // matrix_ident(3, m_repos_R);
        m_repos_R[0] = 1.0; m_repos_R[1] = 0.0; m_repos_R[2] = 0.0;
        m_repos_R[3] = 0.0; m_repos_R[4] = 1.0; m_repos_R[5] = 0.0;
        m_repos_R[6] = 0.0; m_repos_R[7] = 0.0; m_repos_R[8] = 1.0;

        m_repos_d[0] = m_repos_d[1] = m_repos_d[2] = 0.0;
        m_repos_scale = 1.0;

        m_metric = false;

        m_estimate_up_vector_szeliski = false;
    }

    virtual bool OnInit();

    /* 处理命令行选项 */
    virtual void ProcessOptions(int argc, char **argv);

    /* 返回相机参数的数量 */
    int GetNumCameraParameters();

    /* 丰富对应关系 */
    void EnrichCorrespondences(double alpha, double threshold);

    /* 移除低于阈值的匹配 */
    void PruneMatchesThreshold(int threshold);
    /* 移除接近两幅图像边缘的匹配 */
    void RemoveMatchesNearBorder(int i1, int i2, int border_width);
    /* 移除接近图像底部的匹配 */
    void RemoveMatchesNearBottom(int i1, int i2, int border_width);

    /* 计算一对图像的转换关系 */
    bool ComputeTransform(int idx1, int idx2, bool removeBadMatches);

    /* 计算所有匹配的转换关系 */
    void ComputeTransforms(bool removeBadMatches, int new_image_start = 0);

    /* 计算一对图像的对极几何关系 */
    bool ComputeEpipolarGeometry(int idx1, int idx2, bool removeBadMatches);

    /* 计算所有图像的对极几何关系 */
    void ComputeEpipolarGeometry(bool removeBadMatches,
				 int new_image_start = 0);

    /* 计算一组对匹配的解释轨迹 */
    void ComputeTracks(int new_image_start = 0);

    /* 计算一对图像的几何约束关系 */
    void ComputeGeometricConstraints(bool overwrite = false,
				     int new_image_start = 0);

#ifndef __DEMO__
    /* 在相机上设置约束 */
    void SetCameraConstraints(int cam_idx, camera_params_t *params);
    void SetFocalConstraint(const ImageData &data, camera_params_t *params);
    void ClearCameraConstraints(camera_params_t *params);
#endif

    void CheckPointKeyConsistency(const std::vector<ImageKeyVector> pt_views,
                                  int *added_order);

#ifndef __DEMO__
    /* 初始化光速平差法（或从文件中导入） */
    void InitializeBundleAdjust(int &num_init_cams,
				int *added_order,
				int *added_order_inv,
				camera_params_t *cameras,
				v3_t *points, v3_t *colors,
				std::vector<ImageKeyVector> &pt_views,
				bool use_constraints);

    /* 初始化投影矩阵和vmask */
    void SetupProjections(int num_cameras, int num_points,
			  int *added_order,
			  v2_t *projections, char *vmask);

    /* 找到拥有最多匹配数量的相机 */
    int FindCameraWithMostMatches(int num_cameras, int num_points,
				  int *added_order,
				  int &parent_idx, int &max_matches,
				  const std::vector<ImageKeyVector> &pt_views);

    /* 找到所有匹配数量大于N的相机 */
    std::vector<ImagePair> FindCamerasWithNMatches(int n,
						   int num_cameras,
						   int num_points,
						   int *added_order,
						   const std::vector<ImageKeyVector> &pt_views);

    int FindCameraWithMostConnectivity(int num_cameras, int num_points,
				       int *added_order,
				       int &parent_idx,
				       int &max_matches);

    /* 三角化一个子轨迹 */
    v3_t TriangulateNViews(const ImageKeyVector &views,
			   int *added_order, camera_params_t *cameras,
			   double &error, bool explicit_camera_centers);

    v3_t GeneratePointAtInfinity(const ImageKeyVector &views,
                                 int *added_order,
                                 camera_params_t *cameras,
                                 double &error,
                                 bool explicit_camera_centers);

    /* 在光束平差法中加入新点 */
    int BundleAdjustAddNewPoints(int camera_idx,
				 int num_points, int num_cameras,
				 int *added_order,
				 camera_params_t *cameras,
				 v3_t *points, v3_t *colors,
				 double reference_baseline,
				 std::vector<ImageKeyVector> &pt_views);

    /* 在光束平差法中加入新点 */
    int BundleAdjustAddAllNewPoints(int num_points, int num_cameras,
				    int *added_order,
				    camera_params_t *cameras,
				    v3_t *points, v3_t *colors,
				    double reference_baseline,
				    std::vector<ImageKeyVector> &pt_views,
				    double max_reprojection_error = 16.0,
                                    int min_views = 2);

    /* 移除重建中的坏点和相机 */
    int RemoveBadPointsAndCameras(int num_points, int num_cameras,
                                  int *added_order,
                                  camera_params_t *cameras,
                                  v3_t *points, v3_t *colors,
                                  std::vector<ImageKeyVector> &pt_views);

    /* 计算所有相机位姿 */
    void BundleAdjust();

    /* 快速计算所有相机位姿 */
    void BundleAdjustFast();


    /* 估计所有被忽略的相机位姿 */
    void EstimateIgnoredCameras(int &curr_num_cameras,
                                camera_params_t *cameras,
                                int *added_order,
                                int &pt_count,
                                v3_t *points,
                                v3_t *colors,
                                std::vector<ImageKeyVector> &pt_views);

    /* 寻找一对好的相机来初始化光束平差法 */
    void BundlePickInitialPair(int &i_best, int &j_best,
                               bool use_init_focal_only);

    /* 设置相机对来初始化光束平差法 */
    int SetupInitialCameraPair(int i_best, int j_best,
			       double &init_focal_length_0,
			       double &init_focal_length_1,
			       camera_params_t *cameras,
			       v3_t *points, v3_t *colors,
			       std::vector<ImageKeyVector> &pt_views);

    /* 初始化单幅图像 */
    void BundleImage(char *filename, int parent_img);
    /* 从文件中初始化图像 */
    void BundleImagesFromFile(FILE *f);


    /* 从光束平差法中初始化图像 */
    camera_params_t
        BundleInitializeImage(ImageData &data,
                              int image_idx, int camera_idx,
                              int num_cameras, int num_points,
                              int *added_order, v3_t *points,
                              camera_params_t *parent,
                              camera_params_t *cameras,
                              std::vector<ImageKeyVector> &pt_views,
                              bool *success_out = NULL,
			      bool refine_cameras_and_points = false);

    /* 从光束平差法中初始化图像（进行全局优化） */
    void BundleInitializeImageFullBundle(int image_idx, int parent_idx,
					 int num_cameras,
					 int num_points,
					 int *added_order,
					 camera_params_t *cameras,
					 v3_t *points, v3_t *colors,
					 std::vector<ImageKeyVector>
					     &pt_views);


    /* 修正一组3D点 */
    double RefinePoints(int num_points, v3_t *points, v2_t *projs,
			int *pt_idxs, camera_params_t *cameras,
			int *added_order,
			const std::vector<ImageKeyVector> &pt_views,
			camera_params_t *camera_out);

    /* 修正给定相机的观测位置 */
    std::vector<int> RefineCameraAndPoints(const ImageData &data,
                                           int num_points,
					   v3_t *points, v2_t *projs,
					   int *pt_idxs,
					   camera_params_t *cameras,
					   int *added_order,
					   const std::vector<ImageKeyVector>
					      &pt_views,
					   camera_params_t *camera_out,
					     bool remove_outliers);


    void MatchCloseImagesAndAddTracks(ImageData &data, int this_cam_idx,
                                      int added_order_idx,
                                      std::vector<ImageKeyVector> &pt_views);
    void RunSFMWithNewImages(int new_images,
                             double *S = NULL, double *U = NULL, \
                             double *V = NULL, double *W = NULL);
    void ReRunSFM(double *S = NULL, double *U = NULL, double *V = NULL,
                  double *W = NULL);

    /* 在给定的重建结构中运行光束平差法 */
    double RunSFM(int num_pts, int num_cameras, int start_camera,
                  bool fix_points, camera_params_t *init_camera_params,
                  v3_t *init_pts, int *added_order, v3_t *colors,
                  std::vector<ImageKeyVector> &pt_views,
                  int max_iter = 0, int max_iter2 = 0,
                  int verbosity = 0, double eps2 = 1.0e-12,
                  double *S = NULL, double *U = NULL, double *V = NULL,
                  double *W = NULL, bool remove_outliers = true,
                  bool final_bundle = false,
                  bool write_intermediate = false);

    double RunSFM_SBA(int num_pts, int num_cameras, int start_camera,
                      bool fix_points, camera_params_t *init_camera_params,
                      v3_t *init_pts, int *added_order, v3_t *colors,
                      std::vector<ImageKeyVector> &pt_views,
                      double eps2 = 1.0e-12,
                      double *S = NULL, double *U = NULL, double *V = NULL,
                      double *W = NULL, bool remove_outliers = true);

#ifdef __USE_CERES__
    double RunSFM_Ceres(int num_pts, int num_cameras, int start_camera,
                        bool fix_points, camera_params_t *init_camera_params,
                        v3_t *init_pts, int *added_order, v3_t *colors,
                        std::vector<ImageKeyVector> &pt_views,
                        int max_iter = 0, int max_iter2 = 0,
                        int verbosity = 0, double eps2 = 1.0e-12,
                        double *S = NULL, double *U = NULL, double *V = NULL,
                        double *W = NULL, bool remove_outliers = true,
                        bool final_bundle = false,
                        bool write_intermediate = false);
#endif /* __USE_CERES__ */

    double RunSFMNecker(int i1, int i2,
                        camera_params_t *cameras,
                        int num_points, v3_t *points, v3_t *colors,
                        std::vector<ImageKeyVector> &pt_views,
                        camera_params_t *cameras_new,
                        v3_t *points_new,
                        double threshold);


#endif /* __DEMO__ */

    bool BundleTwoFrame(int i1, int i2, TwoFrameModel *model,
                        double &angle_out, int &num_pts_out,
                        bool bundle_from_tracks);
    bool EstimateRelativePose(int i1, int i2,
                              camera_params_t &camera1,
                              camera_params_t &camera2);

    bool EstimateRelativePose2(int i1, int i2,
                               camera_params_t &camera1,
                               camera_params_t &camera2);

    /* 在已有模型中注册图像 */
    bool BundleRegisterImage(ImageData &data, bool init_location);
    void RunBundleServer();


#ifdef __USE_BOOST__
    /* 图像操作 */
    ImageGraph ComputeMSTWorkingGraph(std::vector<int> &interior);
    void PartitionGraph(ImageGraph &graph, std::vector<int> interior);
#endif 

    /* 输出压缩后的bundle文件 */
    void OutputCompressed(const char *ext = "compressed");
    void ScaleFocalLengths(double focal);
    void ScaleFocalLengths(char *focal_file);
    void RotateCameras(char *rotate_file);
    void PruneBadPoints();
    void ZeroDistortionParams();
    void OutputRelativePoses2D(const char *outfile);
    void OutputRelativePoses3D(const char *outfile);
    void ComputeCameraCovariance();

    /* 分析点数据 */
    void AnalyzePoints();

    /* 在场景中找到地平面 */
    void FindGroundPlane();

    /* 在场景中找到天平面 */
    void FindSkyPlane();

    /* 自选举特征点 */
    void CoalesceFeatureDescriptors();
    void CoalesceFeatureDescriptorsMedian();

    /* 计算特征点和重建和的特征点的相似度 */
    std::vector<KeypointMatch>
        MatchKeysToPoints(const std::vector<KeypointWithDesc> &k1,
                          double ratio = 0.6);

    std::vector<KeypointMatch>
        MatchPointsToKeys(const std::vector<KeypointWithDesc> &keys,
			  double ratio = 0.6);

    void ReadProjectivePoints();
    void ReadProjectiveCameras();

    bool ImageVerifiesRay(int img, const double *p0, const double *p1);
    std::vector<int> GetVerifiersForImage(int img);
    void GetPointCoverage(int img, double &left, double &right,
                          double &up, double &down);
    int PredictNextImage(LinkDirection &dir);
    void RenderPredictedImage(int idx, LinkDirection dir,
                              const char *out_file);




    /* **** Bundler 选项 **** */

    bool m_panorama_mode;        
    bool m_add_images_fast;
    bool m_estimate_ignored;
    bool m_analyze_matches;      

    int m_ann_max_pts_visit;     

    bool m_optimize_for_fisheye; 

    int m_homography_rounds;     
    double m_homography_threshold;

    int m_fmatrix_rounds;        
    double m_fmatrix_threshold;
    bool m_skip_fmatrix;
    bool m_skip_homographies;
    bool m_use_angular_score;

    double m_projection_estimation_threshold;  

    double m_min_proj_error_threshold;
    double m_max_proj_error_threshold;

    double m_min_camera_distance_ratio;  

    double m_baseline_threshold;     

    double m_ray_angle_threshold;    

    bool m_use_focal_estimate;       

    bool m_trust_focal_estimate;

    int m_min_max_matches;           

    int m_num_matches_add_camera;    

    const char *m_bundle_output_file;  
    const char *m_bundle_output_base;
    const char *m_output_directory;

    bool m_compute_covariance;   
    int m_covariance_fix1;       
    int m_covariance_fix2;       

    int m_keypoint_border_width; 
    int m_keypoint_border_bottom; 
    double m_init_focal_length;  
    bool m_fixed_focal_length;   
    bool m_use_constraints;     
    bool m_constrain_focal;      
    double m_constrain_focal_weight;  

    bool m_factor_essential;     

    bool m_estimate_distortion;  
    double m_distortion_weight;  

    bool m_construct_max_connectivity; 

    bool m_only_bundle_init_focal;  

    bool m_fix_necker;           

    int m_initial_pair[2];       

    bool m_features_coalesced;   

    bool m_assemble;             
    bool m_run_bundle;           
    bool m_rerun_bundle;         
    bool m_fast_bundle;          

#ifdef __USE_CERES__
    bool m_use_ceres;            
#endif 

    bool m_skip_full_bundle;     
    bool m_skip_add_points;      

    
    bool m_compress_list;        
				  
    bool m_reposition_scene;    
    bool m_prune_bad_points;    



    double m_scale_focal;        

    bool m_predict_next_image;
    char *m_prediction_image;

    char *m_add_image_file;      
    char *m_scale_focal_file;
    char *m_rotate_cameras_file;
    char *m_track_file;

    bool m_output_relposes;
    char *m_output_relposes_file;

    bool m_enrich_points;        
    bool m_zero_distortion_params;
                                   

    int argc;
    char **argv;
};

#endif
