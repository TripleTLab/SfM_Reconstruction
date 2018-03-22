/* Camera.h */
/* Camera classes */

#ifndef __camera_h__
#define __camera_h__

#ifndef __BUNDLER__
#include "Bezier.h"
#include "LinkDirection.h"
#endif

#include "BoundingBox.h"
#include "Geometry.h"

class CameraInfo {
public:
    CameraInfo() {
        m_adjusted = false;

        m_constrained[0] = m_constrained[1] = m_constrained[2] = false;
        m_constrained[3] = m_constrained[4] = m_constrained[5] = false;
        m_constrained[6] = false;

        m_constraints[0] = m_constraints[1] = m_constraints[2] = 0.0;
        m_constraints[3] = m_constraints[4] = m_constraints[5] = 0.0;
        m_constraints[6] = 0.0;

        m_constraint_weights[0] =
            m_constraint_weights[1] =
            m_constraint_weights[2] = 0.0;
        m_constraint_weights[3] =
            m_constraint_weights[4] =
            m_constraint_weights[5] = 0.0;
        m_constraint_weights[6] = 0.0;

        m_k[0] = m_k[1] = 0.0;

#ifndef __BUNDLER__
        for (int i = 0; i < NUM_LINK_DIRECTIONS; i++)
            m_links[i] = -1;
#endif
    }

    /* 相机标定 */
    void Finalize();
    /* 获得相机的刚性变换 */
    void GetRigid(double *T) const;
    /* 获得四阶刚性变换矩阵 */
    void GetRigid4x4(double *T) const;
    /* 返回相机位置 */
    void inline GetPosition(double *pos) const {
        pos[0] = m_R[0] * m_t[0] + m_R[3] * m_t[1] + m_R[6] * m_t[2];
        pos[1] = m_R[1] * m_t[0] + m_R[4] * m_t[1] + m_R[7] * m_t[2];
        pos[2] = m_R[2] * m_t[0] + m_R[5] * m_t[1] + m_R[8] * m_t[2];

        pos[0] = -pos[0];
        pos[1] = -pos[1];
        pos[2] = -pos[2];
    }

    /* 设定相机位置 */
    void SetPosition(const double *pos);
    /* 以旋转矩阵返回相机姿态 */
    void GetPose(double *R) const;
    void GetPoseQuaternion(double *q) const;
    /* 设定相机姿态 */
    void SetPose(const double *R);
    /* 获得垂直旋转矩阵 */
    void GetUprightRotation(int rotate, double *R);
    /* 返回三阶内参矩阵 */
    void GetIntrinsics(double *K) const;
    /* 获得视图视野 */
    double GetFOV() const;
    double GetFOVMax(int rotate) const;
    /* 设定视图视野 */
    void SetFOV(double fov);
    /* 将点投影到相机 */
    bool Project(const double *p, double *proj) const;
    /* 计算两个相机之间的本质矩阵 */
    void ComputeEssentialMatrix(const CameraInfo &cam, double *E, double *F);
    /* 按 Z 轴翻转相机 */
    void Reflect();
    /* 计算两个相机之间的距离 */
    double CameraDistance(const CameraInfo &cam) const;
    /* 寻找图像中的地平线 */
    void ComputeHorizonLine(double *ground, double *up);
    /* 计算消失线 */
    void ComputeVanishingLine(const PlaneData &plane, double *line);
    /* 给定二维点高于地平线时返回 true */
    bool PointInFront(double *p);
    /* 给定点在相机前时返回 true */
    bool PointAboveHorizon(double *p);

#ifndef __BUNDLER__
    /* 返回贝塞尔曲线 */
    Bezier ComputeBezier(const CameraInfo &cam) const;
#endif

    /* Convert a pixel position to a ray direction */
    void PixelToCameraRay(double x, double y, double *ray);
    /* Convert a pixel position to an (absolute) ray direction */
    void PixelToCameraRayAbsolute(double x, double y, double *ray);
    /* Point the camera in a different direction */
    void PointAt(double *ray);
    void PointAtAbsolute(double *ray);
    /* Get the bounding box for this camera */
    BoundingBox GetBoundingBox() const;
    /* Return the view direction */
    void GetViewDirection(double *view) const;
    /* Get the twist angle of the camera */
    double GetTwistAngleRadians() const;
    /* Return the halfspace in front of the camera */
    void GetFrontHalfspace(double *plane) const;
    bool PointInsideImage(const double *p) const;

    /* Get a camera whose up vector points in the right direction */
    CameraInfo GetUpCamera(double *up) const;

#ifndef __BUNDLER__
    /* Read/write the links to a file */
    void ReadLinks(FILE *f);
    void WriteLinks(FILE *f);
#endif

    /* Write in XML format */
    void WriteXML(FILE *f);



    /* Write params to file*/
    void WriteFile(FILE *f);

    bool m_adjusted;        /* Has this camera been adjusted? */
    double m_focal;         /* Focal length */
    double m_k[2];          /* Distortion parameters */
    double m_R[9], m_t[3];  /* Extrinsics */
    double m_Pmatrix[12];
    int m_width, m_height;  /* Image dimensions */

    /* Horizon line */
    double m_horizon[3];

    double m_RGB_transform[12];   /* Local affine transform for RGB
                                   * space */

    double m_color[3];

    /* Constraints on camera center location */
    bool m_constrained[7];
    double m_constraints[7];
    double m_constraint_weights[7];

#ifndef __BUNDLER__
    int m_links[NUM_LINK_DIRECTIONS];
#endif /* __BUNDLER__ */
};

/* Interpolate between two camera views */
CameraInfo InterpolateCameras(const CameraInfo &cam1,
                              const CameraInfo &cam2, double t);

CameraInfo InterpolateCamerasThetaPhi(const CameraInfo &cam1,
                                      const CameraInfo &cam2, double t,
                                      bool interp_fov = false);

#endif /* __camera_h__ */
