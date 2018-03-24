/* ImageData.h */

#ifndef __image_data_h__
#define __image_data_h__
#include <vector>
using namespace std;

#ifndef __BUNDLER__
#include "wx/wx.h"
#endif /* __BUNDLER__ */

#include "BoundingBox.h"
#include "Camera.h"
#include "Geometry.h"
#include "ParameterBound.h"
// #include "Polygon.h"
#include "keys.h"



#include "image.h"



enum ImageBoundary {
    BoundaryNorth,
    BoundaryEast,
    BoundarySouth,
    BoundaryWest,
};

class ImageNote {
public:
    void Read(FILE *f, double width, double height);

    char *m_text;
    BoundingBox m_bbox;
};

class ImageDate {
public:
    ImageDate() : m_known(false) { }

    void GetString(char *buf);
    void GetMonthString(char *buf);

    double GetDateDouble();
    double GetDateOnlyDouble();
    double GetTimeDouble();

    bool m_known;
    int m_month, m_day, m_year;
    int m_hour, m_minute, m_second;
};

int CompareImageDates(ImageDate *date1, ImageDate *date2);

class AnnotationData {
public:
    AnnotationData(int idx, BoundingBox bbox) : m_idx(idx), m_bbox(bbox) { }
    AnnotationData() { }

    int m_idx;
    BoundingBox m_bbox;
};

#ifndef __BUNDLER__
class TPSBasis {
public:
    TPSBasis() {
        m_LU = NULL;
        m_ipiv = NULL;
    }

    void Clear() {
        if (m_LU != NULL) {
            delete [] m_LU;
            m_LU = NULL;
        }

        if (m_ipiv != NULL) {
            delete [] m_ipiv;
            m_ipiv = NULL;
        }

        m_basis_points.clear();
        m_indices.clear();
    }

    double *m_LU;
    int *m_ipiv;
    std::vector<DPoint> m_basis_points;
    std::vector<int> m_indices;
};
#endif /* __BUNDLER__ */

class ImageData {
public:
    ImageData() {
        m_img = NULL;
        m_thumb_fixed = NULL;
        m_thumb = NULL;
        m_thumb256 = NULL;


        m_marked = -1;
        m_canonical=false;
        m_canonical_pano=false;
        m_rahul_pano_index=-1;
        m_texture_index = m_back_texture_index = -1;
        m_thumb_texture_index = -1;
        m_thumb_fixed_texture_index = -1;
        m_texture_image_loaded = false;
        m_panorama_index = -1;
        m_keys_loaded = false;
        m_keys_desc_loaded = false;
        m_keys_scale_rot_loaded = false;
        m_cached_dimensions = false;
        m_cached_keys = false;
        //m_k0 = 1.0;
        //m_k1 = m_k2 = m_k3 = m_k4 = 0.0;
        m_known_intrinsics = false;
        //m_rd_focal = 0.0;
        m_init_focal = 0.0;
        m_has_init_focal = false;
        m_rotation = 0;
        m_is_dragged = false;
        m_is_dropped = false;
        m_ignore_in_bundle = false;
        m_licensed = false;
        m_real_name = NULL;
        m_partition = -1;
        m_layout_x = m_layout_y = 0;
        m_lat = m_long = 0.0;
        m_day_photo = true;
        m_added = false;
        m_is_rep = false;
        m_geosupport = 0.0;
    }

    void InitFromString(char *buf, const char *path,
                        bool fisheye_by_default);

    void UndistortKeys();
    std::vector<Keypoint> UndistortKeysCopy();
    void DistortPoint(double x, double y, double *R,
        double &x_out, double &y_out) const;
    void DistortPoint(double x, double y, double &x_out, double &y_out) const;
    void UndistortPoint(double x, double y,
        double &x_out, double &y_out) const;

    void DistortPointRD(double x, double y,
        double &x_out, double &y_out) const;
    void UndistortPointRD(double x, double y,
        double &x_out, double &y_out) const;

    img_t *UndistortImage(double theta = 0.0, double phi = 0.0,
        int num_rot = 0);
    img_t *UndistortImageResize(int w_new, int h_new);

    void ToNDC(double x, double y, double &x_n, double &y_n);
    void FromNDC(double x_n, double y_n, double &x, double &y);

    //保存图像的尺寸
    void CacheDimensions();
    void CacheNumKeys();

    //返回图像的尺寸
    int GetWidth();
    int GetHeight();
    int GetArea();

    void GetBaseName(char *buf);
    void GetName(char *buf);
    void GetCompressedTextureFilename(char *buf) const;
    bool CompressedTextureExists();

    //返回图像的边界框
    BoundingBox GetBoundingBox();

    v2_t GetPointTexCoords(v2_t point);

    std::vector<v2_t> GetPointTexCoords(const std::vector<v2_t> &pts);

    bool PixelInRange(double x, double y);

    void LoadImage();
    void UnloadImage();

    void LoadTexImage();
    void UnloadTexImage();

	void LoadBackTexImage();
	void UnloadBackTexImage();

	bool TexImageExists();

    bool LoadFeatureWeightMap();
    void SaveFeatureWeightMap();
    void ComputeFeatureWeightMap(int id,
        const std::vector<PointData> &pt_data);
    void ComputeFeatureWeightMap(int id,
        const std::vector<PointData> &pt_data,
        const std::vector<int> &used_points);

    void CheckLoadFloatingThumb();
    void LoadFloatingThumbnail();
    void UnloadFloatingThumbnail();

    void CheckLoadFixedThumb();
    void LoadFixedThumbnail(int w_max, int h_max, int rotation);
    void UnloadFixedThumbnail();

    void LoadThumb256();
    void UnloadThumb256();

    int GetNumKeys();
    void LoadOrExtractKeys(const char *sift_binary, bool undistort = true);
    void LoadKeys(bool descriptor = true, bool undistort = true);
    void LoadDescriptors(bool undistort);
    void LoadKeysWithScaleRot(bool descriptor = true, bool undistort = true);
    void UnloadKeys();
    void UnloadKeysWithScaleRot();
    void ExtractFeatures(const char *sift_binary, bool undistort);

    //查找线段
    void DetectLineSegments(double sigma,
        double threshold1, double threshold2,
        double min_line_segment_size);

    //将线段给图片
    img_t *RenderLineSegments();

    //查询图像是否支持给定线段
    bool LineSegmentSupportedByImage(LineSegment2D &line);

    //判断线是否与图像相交
    bool LineIntersectsImage(double *line, double *isect1, double *isect2,
        ImageBoundary &b1, ImageBoundary &b2);

    //在给定位置查找渐变方向
    void Gradient(double x, double y, double *grad);

    vector<bool> m_visinfo;
    vector<int> m_visindex;


    void ReadKeyColors();

    bool ReadCamera();

    bool ReadTracks(int img_idx, std::vector<PointData> &pt_list);

    void WriteCamera();

    void WriteCameraXML(FILE *f);

    void WriteTracks();

    void ReadMetadata();

    std::vector<ImageNote> GetNotes();

#ifndef __BUNDLER__
    //计算用于估计图像中失真的3D点
    void ComputeDistortionPoints(const std::vector<PointData> &pt_data,
                                 std::vector<DPoint3> &pts_world,
                                 std::vector<DPoint3> &pts_plane);

    void ComputeOrthoPlane(const std::vector<PointData> &pt_data);

    void ComputeTPSBasis(const std::vector<PointData> &pt_data);
    void FreeTPSBasis();
#endif /* __BUNDLER__ */

    void ComputeHistogram(int nbins, double *r_hist,
        double *g_hist, double *b_hist);

    void ComputeHistogramLUV(int nbins, double *L_hist,
        double *U_hist, double *V_hist);

    void ComputeHistogramLUVNorm(int nbins, double *L_hist,
        double *U_hist, double *V_hist);

    int m_width, m_height; 
    int m_xmin,m_xmax,m_ymin,m_ymax; 
    bool m_cached_dimensions;

    int m_num_keys;        
    bool m_cached_keys;

    bool m_added;
    int m_marked; 
    bool m_canonical;
    bool m_canonical_pano;
    int m_rahul_pano_index;

    char *m_real_name;
    char *m_name;             
    char *m_key_name; 
    char m_user_name[256];
    char m_flickr_index[256];
    ImageDate m_date;
    bool m_day_photo;

#ifndef __BUNDLER__
    wxImage *m_wximage;
    wxBitmap *m_bitmap;
#endif /* __BUNDLER__ */

    img_t *m_img;
    img_t *m_texture_img;
	img_t *m_back_texture_img;

    img_t *m_thumb;
    img_t *m_thumb8;
    img_t *m_thumb_fixed;
    img_t *m_thumb256;
    img_t *m_feature_weight_map; 

    bool m_image_loaded, m_keys_loaded, m_keys_desc_loaded,
        m_keys_scale_rot_loaded;
    bool m_texture_image_loaded;
    bool m_licensed;

    bool m_fisheye;
    double m_fCx, m_fCy;
    double m_fRad, m_fAngle;
    double m_fFocal;

    double m_lat, m_long;
    double m_geocentric[3];
    double m_geoplanar[3];
    double m_geosupport;

    std::vector<AnnotationData> m_notes;

    bool m_known_intrinsics;
    double m_K[9];
    //double m_k0, m_k1, m_k2, m_k3, m_k4, m_rd_focal;
    double m_k[5];

    bool m_ignore_in_bundle;

    bool m_has_init_focal;
    double m_init_focal;

    CameraInfo m_camera;

    std::vector<LineSegment2D> m_line_segments;

    int m_texture_index;
	int m_back_texture_index;
    int m_thumb_texture_index; 
    int m_thumb_fixed_texture_index;

    ParameterBound m_bounds;
    ParameterBound m_thumb_bounds;
    ParameterBound m_thumb_fixed_bounds;

    PlaneData m_fit_plane;
    PlaneData m_current_plane;

    std::vector<Keypoint> m_keys;
    std::vector<KeypointWithDesc> m_keys_desc;
    std::vector<KeypointWithScaleRot> m_keys_scale_rot;
    std::vector<bool> m_key_flags;

    std::vector<int> m_visible_points;
    std::vector<int> m_visible_keys;

    std::vector<int> m_visible_lines;

#ifndef __BUNDLER__
    std::vector<DPoint3> m_dist_pts_world;
    std::vector<DPoint3> m_dist_pts_plane;
#endif /* __BUNDLER__ */

    std::vector<int> m_neighbors;

    int m_rotation;
    int m_panorama_index;

    bool m_is_dragged;
    bool m_is_dropped;

    bool m_is_rep;

    int m_drag_x, m_drag_y;

    double m_drop_pt[3]; 
    int m_partition;
    int m_layout_x, m_layout_y;
	
#ifndef __BUNDLER__
    TPSBasis m_tps_basis;
#endif
};

#endif /* __image_data_h__ */
