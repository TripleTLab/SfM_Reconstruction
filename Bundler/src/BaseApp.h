/* BaseApp.h */
/* Base application */

#ifndef __baseapp_h__
#define __baseapp_h__


#ifndef __USE_ANN__
#include "BruteForceSearch.h"
#else
#include "anniface.h"
#endif 

#include "ImageData.h"


#ifndef __DEMO__
#include "sfm.h"
#endif

#include "defines.h"

#include <assert.h>
#include <algorithm>
#include <list>

#ifndef WIN32
#include <unordered_map>
#include <unordered_set>
#else
#include <hash_map>
#include <hash_set>
#endif


#ifdef __USE_BOOST__
#include "boost/graph/graph_traits.hpp"
#include "boost/graph/adjacency_list.hpp"
using namespace boost;
#endif

#ifdef __USE_BOOST__
typedef adjacency_list<vecS, vecS,
                       undirectedS,
                       property<vertex_color_t, int>,
                       property<edge_weight_t, int> > ImageGraph;
#endif

class TransformInfo {
public:

    void ReadFromFile(FILE *f);
    void WriteToFile(FILE *f);

    double m_fmatrix[9];
    double m_ematrix[9];

    double m_H[9];
    double m_inlier_ratio;
    int m_num_inliers;

    double m_gain[3], m_bias[3];
};

typedef std::pair<unsigned long, unsigned long> MatchIndex;

#ifdef WIN32
namespace stdext {
    template<>
    class hash_compare<MatchIndex> {
	public:
		static const size_t bucket_size = 4;
        static const size_t min_buckets = 8;
        size_t
        operator()(const MatchIndex &__x) const
        { return __x.first * 1529 + __x.second; }

        bool operator()(const MatchIndex &__x1, const MatchIndex &__x2) const {
			return (__x1.first < __x2.first) || (__x1.first == __x2.first && __x1.second < __x2.second);
        }
    };
}
#else
namespace std {
    template<>
    struct hash<MatchIndex> {
        size_t
        operator()(MatchIndex __x) const
        { return __x.first * 1529 + __x.second; }
    };
}
#endif

#ifdef WIN32
typedef stdext::hash_map<unsigned int, std::vector<KeypointMatch> >
   MatchAdjTable;
#else
typedef unordered_map<unsigned int, std::vector<KeypointMatch> >
   MatchAdjTable;
#endif

class AdjListElem {
public:
    bool operator< (const AdjListElem &other) const {
        return m_index < other.m_index;
    }

    unsigned int m_index;
    std::vector<KeypointMatch> m_match_list;
};

typedef std::vector<AdjListElem> MatchAdjList;

class MatchTable
{
public:

    MatchTable() { }

    MatchTable(int num_images) {
        m_match_lists.resize(num_images);
    }

    void SetMatch(MatchIndex idx) {
        if (Contains(idx))
            return; 
#if 0
        MatchAdjList tmp;
        AdjListElem adjlist_elem;
        adjlist_elem.m_index = idx.second;
        tmp.push_back(adjlist_elem);
        m_match_lists[idx.first].merge(tmp);
#else
        /* Using vector */
        AdjListElem e;
        e.m_index = idx.second;
        MatchAdjList &l = m_match_lists[idx.first];
        MatchAdjList::iterator p = lower_bound(l.begin(), l.end(), e);
        l.insert(p, e);
#endif
    }

    void AddMatch(MatchIndex idx, KeypointMatch m) {
        assert(Contains(idx));
        GetMatchList(idx).push_back(m);
    }

    void ClearMatch(MatchIndex idx) {
        if (Contains(idx)) {
            GetMatchList(idx).clear();
        }
    }

    void RemoveMatch(MatchIndex idx) {
        if (Contains(idx)) {
            std::vector<KeypointMatch> &match_list = GetMatchList(idx);
            match_list.clear();

#if 0
            
#endif
            AdjListElem e;
            e.m_index = idx.second;
            MatchAdjList &l = m_match_lists[idx.first];
            std::pair<MatchAdjList::iterator, MatchAdjList::iterator> p =
                equal_range(l.begin(), l.end(), e);

            assert(p.first != p.second);

            l.erase(p.first, p.second);
        }
    }

    unsigned int GetNumMatches(MatchIndex idx) {
        if (!Contains(idx))
            return 0;
        return GetMatchList(idx).size();
    }

    std::vector<KeypointMatch> &GetMatchList(MatchIndex idx) {
       
        AdjListElem e;
        e.m_index = idx.second;
        MatchAdjList &l = m_match_lists[idx.first];
        std::pair<MatchAdjList::iterator, MatchAdjList::iterator> p =
            equal_range(l.begin(), l.end(), e);

        assert(p.first != p.second);

        return (p.first)->m_match_list;
    }

    bool Contains(MatchIndex idx) const {
        AdjListElem e;
        e.m_index = idx.second;
        const MatchAdjList &l = m_match_lists[idx.first];
        std::pair<MatchAdjList::const_iterator,
            MatchAdjList::const_iterator> p =
            equal_range(l.begin(), l.end(), e);

        return (p.first != p.second);
    }

    void RemoveAll() {
        int num_lists = m_match_lists.size();

        for (int i = 0; i < num_lists; i++) {
            m_match_lists[i].clear();
        }
    }

    unsigned int GetNumNeighbors(unsigned int i) {
        return m_match_lists[i].size();
    }

#if 0
    
#endif

    MatchAdjList &GetNeighbors(unsigned int i) {
        return m_match_lists[i];
    }

    MatchAdjList::iterator Begin(unsigned int i) {
        return m_match_lists[i].begin();
    }

    MatchAdjList::iterator End(unsigned int i) {
        return m_match_lists[i].end();
    }

private:
    std::vector<MatchAdjList> m_match_lists;
};

/* 返回图像匹配的索引 */
MatchIndex GetMatchIndex(int i1, int i2);
MatchIndex GetMatchIndexUnordered(int i1, int i2);
// #include "GetMatchIndex.h"

class BaseApp
{
public:
    virtual ~BaseApp() { }

    virtual bool OnInit() = 0;

    /* 处理命令行选项 */
    virtual void ProcessOptions(int argc, char **argv) = 0;

    /* 返回图片数量 */
    int GetNumImages();
    int GetNumOriginalImages();
    int GetNumMatches(int i1, int i2);

    int FindImageWithName(const char *name);

    /* 获取匹配信息 */
    void SetMatch(int i1, int i2);
    void RemoveMatch(int i1, int i2);
    bool ImagesMatch(int i1, int i2);

    /* 获取特征点 */
    Keypoint &GetKey(int img, int key);
    KeypointWithDesc &GetKeyWithDesc(int img, int key);
    int GetNumKeys(int img);
    /* 获取相机与对应图像的索引 */
    int GetRegisteredCameraIndex(int cam);

    /* 通过文件加载图片名列表 */
    void LoadImageNamesFromFile(FILE *f);

    /* 通过文件加载匹配信息 */
    void LoadMatches();
    void ReadMatchFile(int i, int j);
    void LoadMatchTable(const char *filename);
    void LoadMatchIndexes(const char *index_dir);
    /* 通过文件加载特征点 */
    void LoadKeys(bool descriptor = true);
    void RemoveAllMatches();
    /* 删除同时匹配多个图像的特征点 */
    void PruneDoubleMatches();

    /* 重排序匹配列表使其相对称 */
    void MakeMatchListsSymmetric();

    /* 初始化存储每幅图像的点和线的空间 */
    void SetupImagePoints(int min_views = 1);

    void ReadGeometricConstraints(const char *filename);
    void WriteGeometricConstraints(const char *filename);
    void WriteTracks(char *filename);
    void WriteTracks2(char *filename);
    void ReadCameraConstraints();
    void ReadPointConstraints();
    void ReadIntrinsicsFile();
    bool ReadTrackPairs(const char *filename);
    void WriteTrackPairs(const char *filename);

    /* 读取忽略的文件 */
    void ReadIgnoreFile();

    /* 读取特征点的颜色信息 */
    void ReadKeyColors();

    /* 读取世界坐标系的信息 */
    void ReadBundleFile(const char *filename);
    void ReloadBundleFile (const char *filename);

    /* 读取/写入线 */
    void ReadLines3D(const char *filename);
    void WriteLines3D(const char *filename);

    /* 清除当前的模型 */
    void ClearModel();

    /* 读取/写入匹配列表 */
    void ReadMatchTable(const char *append = "");
    void WriteMatchTable(const char *append = "");

    /* 通过文件初始化未经过调整的图像 */
    void InitializeImagesFromFile(FILE *f);

    void SetTracks(int image);
    /* 设置相机轨迹 */
    void CreateTracksFromPoints();
    void SetTracksFromPoints();
    int SetTracksFromPoints(int image);
    /* 通过轨迹设置匹配信息 */
    void SetMatchesFromTracks();
    void SetMatchesFromTracks(int img1, int img2);
    int GetNumTrackMatches(int img1, int img2);

    /* 通过全局调整设置新的匹配信息 */
    void SetMatchesFromPoints(int threshold = 0);

    void ReindexPoints();

    /* 建立对相机的搜索树 */
#ifdef __USE_ANN__
    ANNkd_tree *CreateCameraSearchTree();
#else
    BruteForceSearch *CreateCameraSearchTree();
#endif

#ifndef __DEMO__
    /* 将点云信息写入ply文件 */
    void DumpPointsToPly(const char *output_directory, const char *filename,
                         int num_points, int num_cameras,
			 v3_t *points, v3_t *colors, camera_params_t *cameras
                         /*bool reflect = true*/);

    /* 输出包含当前世界状态的文件 */
    void DumpOutputFile(const char *output_dir, const char *filename,
			int num_images, int num_cameras, int num_points,
			int *added_order,
			camera_params_t *cameras, v3_t *points, v3_t *colors,
			std::vector<ImageKeyVector> &pt_views
                        /*bool output_radial_distortion = false*/);

#endif

    void WritePointsXML(const char *filename);
    void WritePointsGeoXML(const char *filename);
    void WriteCamerasXML(const char *filename);
    void WriteCamerasGeoXML(const char *filename);

    /* 向文件写入点云 */
    void WritePoints(const char* filename);

    /* 想文件写入相机参数 */
    void WriteCameras(const char* filename);

    /* 修整相机反射 */

    void RemoveBadImages(int min_num_points);

    /* 估计点法向量的置信度 */
    void EstimatePointNormalsConfidence();
    void EstimatePointNormals();

    /* 根据图像旋转向量估计坐标轴 */
    void EstimateAxes(double *xaxis, double *yaxis, double *zaxis);
    /* 分析场景信息 */
    void SetupScene(double *center, double *up,
		    double *x_axis, double *z_axis, double &scale);

    void SetupSceneGroundPlane(double *center, double *up,
                               double *x_axis, double *z_axis, double &scale);

    void TransformWorldReal();
    virtual void RepositionScene(double *center_out, double *R_out,
                                 double &scale_out);
    void TransformSceneCanonical(int c1, int c2);
    void UnscaleCameras(int start_camera);

    /* 计算每幅图的旋转向量 */
    void ComputeImageRotations();

    bool ImagesPartOfPanorama(int i1, int i2);


    double m_bundle_version;

    /* 地理数据 */

    std::vector<ImageData> m_image_data;   /* 图像数据 */
    int m_num_original_images;

    std::vector<PointData> m_point_data;   /* 场景中的3D点云信息 */

    std::vector<int> m_num_views_orig;

    std::vector<TrackData> m_track_data;   /* 检测到的3D轨迹信息 */

    ImageKeyVector m_outliers;             /* 检测到的离群特征点 */



    PlaneData m_ground_plane;        /* 地平面 */

    double m_repos_R[9];
    double m_repos_d[3];
    double m_repos_scale;

    double m_xform[16];       /* 场景转换 */

    /* 点云约束信息 */
    bool m_use_point_constraints;
    v3_t *m_point_constraints;
    double m_point_constraint_weight;
    char *m_point_constraint_file;

    bool m_matches_loaded;
    bool m_matches_computed;
    MatchTable m_matches;

#if 0

#endif

#ifndef WIN32
    unordered_map<MatchIndex, TransformInfo> m_transforms;
#else
    stdext::hash_map<MatchIndex, TransformInfo> m_transforms;
#endif


    double m_scale;              /* 场景尺度 */
    bool m_metric;               /* 是否以米为度量标准 */

    bool m_fisheye;              /* 是否鱼眼图像 */
    char *m_fisheye_params;      /* 鱼眼参数文件 */

    char *m_ignore_file;         /* 保存全局调整中被忽略的文件 */
    bool m_use_intrinsics;
    char *m_intrinsics_file;     /* 使用内参矩阵的文件 */

    bool m_bundle_provided;      /* 全局调整文件是否存在 */
    char *m_bundle_file;         /* 全局文件 */

    const char *m_match_directory;     /* 匹配信息的存储路径 */
    const char *m_match_index_dir;     /* 匹配索引的存储路径 */
    const char *m_match_table;         /* 匹配列表的存储路径 */
    const char *m_key_directory;
    const char *m_image_directory;
    const char *m_sift_binary;         /* sift文件的路径 */

    bool m_estimate_up_vector_szeliski; 

    int m_min_track_views;           /* 每条轨迹的最少观测点 */
    int m_max_track_views;           /* 每条轨迹的最大观测点 */

    int m_min_num_feat_matches;      /* 一对图像的最少匹配点数量 */

    int m_up_image;                  

    int m_start_camera;              

#ifdef __USE_BOOST__
    ImageGraph m_image_graph;
    ImageGraph m_working_image_graph;
    std::vector<int> m_working_images;

    std::vector<int> m_graph_components;
    int m_max_graph_component;
#endif 
};

#endif 
