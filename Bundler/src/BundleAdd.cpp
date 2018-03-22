/* BundleAdd.cpp */
//添加新点

#include <float.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#ifndef WIN32
#include <map>
#else
#include <hash_map>
#endif

#ifdef WIN32
#define isnan _isnan
#endif

#include "defines.h"
#include "matrix.h"
#include "triangulate.h"
#include "util.h"

#include "BundlerApp.h"
#include "Bundle.h"
#include "Distortion.h"

#define INIT_REPROJECTION_ERROR 16.0 /* 6.0 */ /* 8.0 */
#define ADD_REPROJECTION_ERROR 16.0 /* 1.0e2 */ /* 8.0 */ /* 4.0 */

v3_t BundlerApp::TriangulateNViews(const ImageKeyVector &views,
                                   int *added_order,
                                   camera_params_t *cameras,
                                   double &error,
                                   bool explicit_camera_centers)
{
    int num_views = (int) views.size();

    v2_t *pv = new v2_t[num_views];
    double *Rs = new double[9 * num_views];
    double *ts = new double[3 * num_views];

    for (int i = 0; i < num_views; i++) {
	camera_params_t *cam = NULL;

	int camera_idx = views[i].first;
	int image_idx = added_order[camera_idx];
	int key_idx = views[i].second;
	Keypoint &key = GetKey(image_idx, key_idx);

	double p3[3] = { key.m_x, key.m_y, 1.0 };

        if (m_optimize_for_fisheye) {
            double x = p3[0], y = p3[1];
            m_image_data[image_idx].UndistortPoint(x, y, p3[0], p3[1]);
        }

	double K[9], Kinv[9];
	GetIntrinsics(cameras[camera_idx], K);
	matrix_invert(3, K, Kinv);

	double p_n[3];
	matrix_product(3, 3, 3, 1, Kinv, p3, p_n);

	pv[i] = v2_new(-p_n[0], -p_n[1]);
        pv[i] = UndistortNormalizedPoint(pv[i], cameras[camera_idx]);

	cam = cameras + camera_idx;

	memcpy(Rs + 9 * i, cam->R, 9 * sizeof(double));
	if (!explicit_camera_centers) {
	    memcpy(ts + 3 * i, cam->t, 3 * sizeof(double));
	} else {
	    matrix_product(3, 3, 3, 1, cam->R, cam->t, ts + 3 * i);
	    matrix_scale(3, 1, ts + 3 * i, -1.0, ts + 3 * i);
	}
    }

    v3_t pt = triangulate_n(num_views, pv, Rs, ts, &error);

    error = 0.0;
    for (int i = 0; i < num_views; i++) {
	int camera_idx = views[i].first;
	int image_idx = added_order[camera_idx];
	int key_idx = views[i].second;
	Keypoint &key = GetKey(image_idx, key_idx);

	v2_t pr = sfm_project_final(cameras + camera_idx, pt,
				    explicit_camera_centers ? 1 : 0,
                                    m_estimate_distortion ? 1 : 0);

        if (m_optimize_for_fisheye) {
            double x = Vx(pr), y = Vy(pr);
            m_image_data[image_idx].DistortPoint(x, y, Vx(pr), Vy(pr));
        }

	double dx = Vx(pr) - key.m_x;
	double dy = Vy(pr) - key.m_y;

	error += dx * dx + dy * dy;
    }

    error = sqrt(error / num_views);

    delete [] pv;
    delete [] Rs;
    delete [] ts;

    return pt;
}

v3_t BundlerApp::GeneratePointAtInfinity(const ImageKeyVector &views,
                                         int *added_order,
                                         camera_params_t *cameras,
                                         double &error,
                                         bool explicit_camera_centers)
{
    camera_params_t *cam = NULL;

    int camera_idx = views[0].first;
    int image_idx = added_order[camera_idx];
    int key_idx = views[0].second;
    Keypoint &key = GetKey(image_idx, key_idx);

    cam = cameras + camera_idx;

    double p3[3] = { key.m_x, key.m_y, 1.0 };

    if (m_optimize_for_fisheye) {
        double x = p3[0], y = p3[1];
        m_image_data[image_idx].UndistortPoint(x, y, p3[0], p3[1]);
    }

    double K[9], Kinv[9];
    GetIntrinsics(cameras[camera_idx], K);
    matrix_invert(3, K, Kinv);

    double ray[3];
    matrix_product(3, 3, 3, 1, Kinv, p3, ray);

    double ray_world[3];
    matrix_transpose_product(3, 3, 3, 1, cam->R, ray, ray_world);

    double pos[3] = { 0.0, 0.0, 0.0 };
    double pt_inf[3] = { 0.0, 0.0, 0.0 };

    if (!explicit_camera_centers) {

    } else {
        memcpy(pos, cam->t, 3 * sizeof(double));
        double ray_extend[3];
        matrix_scale(3, 1, ray, 100.0, ray_extend);
        matrix_sum(3, 1, 3, 1, pos, ray, pt_inf);
    }

    return v3_new(pt_inf[0], pt_inf[1], pt_inf[2]);
}

static void PrintTrack(const ImageKeyVector &track)
{
    int num_views = (int) track.size();

    printf("[");
    for (int i = 0; i < num_views; i++) {
	printf(" (%d %d)", track[i].first, track[i].second);
	if (i < num_views - 1)
	    printf(",");
    }
    printf(" ]");
}

//添加新点
int
BundlerApp::BundleAdjustAddAllNewPoints(int num_points, int num_cameras,
                                        int *added_order,
                                        camera_params_t *cameras,
                                        v3_t *points, v3_t *colors,
                                        double reference_baseline,
                                        std::vector<ImageKeyVector> &pt_views,
                                        double max_reprojection_error,
                                        int min_views)
{
    std::vector<int> track_idxs;
    std::vector<ImageKeyVector> new_tracks;

    int num_tracks_total = (int) m_track_data.size();
    int *tracks_seen = new int[num_tracks_total];
    for (int i = 0; i < num_tracks_total; i++) {
	tracks_seen[i] = -1;
    }

    for (int i = 0; i < num_cameras; i++) {
	int image_idx1 = added_order[i];

	int num_keys = GetNumKeys(image_idx1);

	for (int j = 0; j < num_keys; j++) {
	    Keypoint &key = GetKey(image_idx1, j);

	    if (key.m_track == -1)
		continue; 

	    if (key.m_extra != -1)
		continue;

	    int track_idx = key.m_track;

	    //检察
	    if (m_track_data[track_idx].m_extra != -1)
		continue;

	    //是否重复检察
	    int seen = tracks_seen[track_idx];

	    if (seen == -1) {
		//没有找到，重新创建
		tracks_seen[track_idx] = (int) new_tracks.size();

		ImageKeyVector track;
		track.push_back(ImageKey(i, j));
		new_tracks.push_back(track);
		track_idxs.push_back(track_idx);
	    } else {
		new_tracks[seen].push_back(ImageKey(i, j));
	    }
	}
    }

    delete [] tracks_seen;

    int pt_count = num_points;

    int num_ill_conditioned = 0;
    int num_high_reprojection = 0;
    int num_cheirality_failed = 0;
    int num_added = 0;

    int num_tracks = (int) new_tracks.size();
    for (int i = 0; i < num_tracks; i++) {
	int num_views = (int) new_tracks[i].size();

	if (num_views < min_views) continue;  //视图不足

#if 0
	printf("Triangulating track ");
	PrintTrack(new_tracks[i]);
	printf("\n");
#endif

	//检察是否有至少两个位置的图片进行了修复
	bool conditioned = false;
	bool good_distance = false;
	double max_angle = 0.0;
	for (int j = 0; j < num_views; j++) {
	    for (int k = j+1; k < num_views; k++) {
		int camera_idx1 = new_tracks[i][j].first;
		int image_idx1 = added_order[camera_idx1];
		int key_idx1 = new_tracks[i][j].second;

		int camera_idx2 = new_tracks[i][k].first;
		int image_idx2 = added_order[camera_idx2];
		int key_idx2 = new_tracks[i][k].second;

		Keypoint &key1 = GetKey(image_idx1, key_idx1);
		Keypoint &key2 = GetKey(image_idx2, key_idx2);

		v2_t p = v2_new(key1.m_x, key1.m_y);
		v2_t q = v2_new(key2.m_x, key2.m_y);

                if (m_optimize_for_fisheye) {
                    double p_x = Vx(p), p_y = Vy(p);
                    double q_x = Vx(q), q_y = Vy(q);

                    m_image_data[image_idx1].
                        UndistortPoint(p_x, p_y, Vx(p), Vy(p));
                    m_image_data[image_idx2].
                        UndistortPoint(q_x, q_y, Vx(q), Vy(q));
                }

		double angle = ComputeRayAngle(p, q,
					       cameras[camera_idx1],
					       cameras[camera_idx2]);

		if (angle > max_angle)
		    max_angle = angle;

		if (RAD2DEG(angle) >= m_ray_angle_threshold) {
		    conditioned = true;
		}

#if 0
		double dist_jk =
		    GetCameraDistance(cameras + j, cameras + k,
				      m_explicit_camera_centers);

		if (dist_jk > m_min_camera_distance_ratio * reference_baseline)
		    good_distance = true;
#else
                good_distance = true;
#endif
	    }
	}

	if (!conditioned || !good_distance) {
	    num_ill_conditioned++;

#if 0
	    printf(">> Track is ill-conditioned [max_angle = %0.3f]\n",
		   RAD2DEG(max_angle));
	    fflush(stdout);
#endif
	    continue;
	}

	double error;
	v3_t pt;

        if (!m_panorama_mode) {
            pt = TriangulateNViews(new_tracks[i], added_order, cameras,
                                   error, true);
        } else {
            pt = GeneratePointAtInfinity(new_tracks[i], added_order, cameras,
                                         error, true);
        }

	if (std::isnan(error) || error > max_reprojection_error) {
	    num_high_reprojection++;
#if 0
	    printf(">> Reprojection error [%0.3f] is too large\n", error);
	    fflush(stdout);
#endif
	    continue;
	}

	bool all_in_front = true;
	for (int j = 0; j < num_views; j++) {
	    int camera_idx = new_tracks[i][j].first;
	    bool in_front = CheckCheirality(pt, cameras[camera_idx]);

	    if (!in_front) {
		all_in_front = false;
		break;
	    }
	}

	if (!all_in_front) {
	    num_cheirality_failed++;

#if 0
	    printf(">> Cheirality check failed\n");
	    fflush(stdout);
#endif
	    continue;
	}

	//通过测试，开始添加点
#if 0
	printf("Triangulating track ");
	PrintTrack(new_tracks[i]);
	printf("\n");
	printf(">> All tests succeeded [%0.3f, %0.3f] for point [%d]\n",
	       RAD2DEG(max_angle), error, pt_count);
#endif

	fflush(stdout);

	points[pt_count] = pt;

	int camera_idx = new_tracks[i][0].first;
	int image_idx = added_order[camera_idx];
	int key_idx = new_tracks[i][0].second;

	unsigned char r = GetKey(image_idx, key_idx).m_r;
	unsigned char g = GetKey(image_idx, key_idx).m_g;
	unsigned char b = GetKey(image_idx, key_idx).m_b;
	colors[pt_count] = v3_new((double) r, (double) g, (double) b);

	pt_views.push_back(new_tracks[i]);

	//关键点设置索引
	for (int j = 0; j < num_views; j++) {
	    int camera_idx = new_tracks[i][j].first;
	    int image_idx = added_order[camera_idx];
	    int key_idx = new_tracks[i][j].second;
	    GetKey(image_idx, key_idx).m_extra = pt_count;
	}

	int track_idx = track_idxs[i];
	m_track_data[track_idx].m_extra = pt_count;

	pt_count++;
        num_added++;
    }

    printf("[AddAllNewPoints] Added %d new points\n", num_added);
    printf("[AddAllNewPoints] Ill-conditioned tracks: %d\n",
           num_ill_conditioned);
    printf("[AddAllNewPoints] Bad reprojections: %d\n", num_high_reprojection);
    printf("[AddAllNewPoints] Failed cheirality checks: %d\n",
           num_cheirality_failed);

    return pt_count;
}

v3_t Triangulate(v2_t p, v2_t q,
                 camera_params_t c1, camera_params_t c2,
                 double &proj_error, bool &in_front, double &angle,
                 bool explicit_camera_centers)
{
    double K1[9], K2[9];
    double K1inv[9], K2inv[9];

    GetIntrinsics(c1, K1);
    GetIntrinsics(c2, K2);

    matrix_invert(3, K1, K1inv);
    matrix_invert(3, K2, K2inv);

    //设置3D的点
    double proj1[3] = { Vx(p), Vy(p), -1.0 };
    double proj2[3] = { Vx(q), Vy(q), -1.0 };

    double proj1_norm[3], proj2_norm[3];

    matrix_product(3, 3, 3, 1, K1inv, proj1, proj1_norm);
    matrix_product(3, 3, 3, 1, K2inv, proj2, proj2_norm);

    v2_t p_norm = v2_new(proj1_norm[0] / proj1_norm[2],
			 proj1_norm[1] / proj1_norm[2]);

    v2_t q_norm = v2_new(proj2_norm[0] / proj2_norm[2],
			 proj2_norm[1] / proj2_norm[2]);

    p_norm = UndistortNormalizedPoint(p_norm, c1);
    q_norm = UndistortNormalizedPoint(q_norm, c2);

    angle = ComputeRayAngle(p, q, c1, c2);

    v3_t pt;
    if (!explicit_camera_centers) {
	pt = triangulate(p_norm, q_norm, c1.R, c1.t, c2.R, c2.t, &proj_error);
    } else {
	double t1[3];
	double t2[3];

	matrix_product(3, 3, 3, 1, c1.R, c1.t, t1);
	matrix_scale(3, 1, t1, -1.0, t1);
	matrix_product(3, 3, 3, 1, c2.R, c2.t, t2);
	matrix_scale(3, 1, t2, -1.0, t2);

	pt = triangulate(p_norm, q_norm, c1.R, t1, c2.R, t2, &proj_error);
    }

    proj_error = (c1.f + c2.f) * 0.5 * sqrt(proj_error * 0.5);

    bool cc1 = CheckCheirality(pt, c1);
    bool cc2 = CheckCheirality(pt, c2);

    in_front = (cc1 && cc2);

    return pt;
}

//添加新点
int BundlerApp::BundleAdjustAddNewPoints(int camera_idx,
                                         int num_points, int num_cameras,
                                         int *added_order,
                                         camera_params_t *cameras,
                                         v3_t *points, v3_t *colors,
                                         double reference_baseline,
                                         std::vector<ImageKeyVector> &pt_views)
{
    int pt_count = num_points;

    int image_idx = added_order[camera_idx];

    for (int i = 0; i < num_cameras; i++) {
	int other = added_order[i];

	if (other == image_idx)
	    continue;

	int first = MIN(image_idx, other);
	int second = MAX(image_idx, other);

	MatchIndex idx = GetMatchIndex(first, second);

        SetMatchesFromTracks(first, second);

	printf("  Matches[%d,%d] = %d\n", image_idx, other,
               (int) m_matches.GetNumMatches(idx));

	double disti = GetCameraDistance(cameras + i, cameras + camera_idx);

	printf("  dist0, disti = %0.3e, %0.3e\n", reference_baseline, disti);

	if (disti < m_min_camera_distance_ratio * reference_baseline) {
	    printf("  Distance too low (possible panorama?)\n");
            m_matches.ClearMatch(idx);
	    continue;
	}

        std::vector<KeypointMatch> &list = m_matches.GetMatchList(idx);
	for (int j = 0; j < (int) list.size(); j++) {
	    int idx1 = list[j].m_idx1;
	    int idx2 = list[j].m_idx2;

	    int this_idx, other_idx;
	    if (image_idx == first) {
		this_idx = idx1;
		other_idx = idx2;
	    } else {
		other_idx = idx1;
		this_idx = idx2;
	    }

	    if (GetKey(other,other_idx).m_extra == -2) {
		continue;
	    } else if (GetKey(image_idx,this_idx).m_extra == -2) {
		continue;
	    }

	    if (GetKey(other,other_idx).m_extra == -1 &&
		GetKey(image_idx,this_idx).m_extra >= 0) {

		/**** 连接现有点 *** */

		int pt_idx = GetKey(image_idx,this_idx).m_extra;

		//检察重投影是否错误
		v2_t pr = sfm_project_final(cameras + i, points[pt_idx],
					    true, m_estimate_distortion);

		double dx = GetKey(other,other_idx).m_x - Vx(pr);
		double dy = GetKey(other,other_idx).m_y - Vy(pr);

		double proj_error = sqrt(dx * dx + dy * dy);

		if (proj_error >= 32.0) {
		    printf("  Would have connected existing match "
			   "%d ==> %d [%d] (cam: %d), \n"
			   "    but reprojection error (%0.3f) "
			   "is too high.\n",
			   this_idx, other_idx, pt_idx, other, proj_error);
		} else {
		    printf("  Connecting existing match "
			   "%d ==> %d [%d] (cam: %d) [%0.3f]\n",
			   this_idx, other_idx, pt_idx, other, proj_error);

		    GetKey(other,other_idx).m_extra = pt_idx;
		    pt_views[pt_idx].push_back(ImageKey(i, other_idx));
		}
	    } else if (GetKey(other,other_idx).m_extra == -1) {

		if (GetKey(image_idx,this_idx).m_extra != -1) {
		    printf("Error!  Key (%d,%d) shouldn't be seen yet!\n",
			   image_idx, this_idx);
		    printf("Point index is %d\n",
			   GetKey(image_idx,this_idx).m_extra);
		}

		//新点
		GetKey(other,other_idx).m_extra = pt_count;
		GetKey(image_idx,this_idx).m_extra = pt_count;

		//建立3D点
		v2_t p = v2_new(GetKey(other,other_idx).m_x,
				GetKey(other,other_idx).m_y);

		v2_t q = v2_new(GetKey(image_idx,this_idx).m_x,
				GetKey(image_idx,this_idx).m_y);

                if (m_optimize_for_fisheye) {
                    double p_x = Vx(p), p_y = Vy(p);
                    double q_x = Vx(q), q_y = Vy(q);

                    m_image_data[other].
                        UndistortPoint(p_x, p_y, Vx(p), Vy(p));
                    m_image_data[image_idx].
                        UndistortPoint(q_x, q_y, Vx(q), Vy(q));
                }

		double proj_error = 0.0;
		bool in_front = false;
		double angle = 0.0;

		points[pt_count] =
		    Triangulate(p, q, cameras[i], cameras[camera_idx],
				proj_error, in_front, angle, true);

		if (RAD2DEG(angle) < m_ray_angle_threshold) {
		    printf(" Ray angle %d => %d is too small (%0.3f)\n",
			   this_idx, other_idx, RAD2DEG(angle));

		    //删除点
		    GetKey(other,other_idx).m_extra = -1;
		    GetKey(image_idx,this_idx).m_extra = -1;

		    continue;
		}

		//检察重投影是否出错
		if (proj_error >= ADD_REPROJECTION_ERROR) {
		    printf("  Projection error for %d => %d is %0.3e, "
			   "removing\n",
			   this_idx, other_idx, proj_error);

		    //删除点
		    GetKey(other,other_idx).m_extra = -2;
		    GetKey(image_idx,this_idx).m_extra = -2;

		    continue;
		}

		if (!in_front) {
		    printf("  Cheirality violated!\n");

		    //删除点
		    GetKey(other,other_idx).m_extra = -2;
		    GetKey(image_idx,this_idx).m_extra = -2;

		    continue;
		}

		printf("  Adding match %d ==> %d [%d] (cam: %d ==> %d) "
		       "[%0.3f, %0.3f]\n",
		       other_idx, this_idx, pt_count, image_idx, other,
		       RAD2DEG(angle), proj_error);

		//添加点
		unsigned char r = GetKey(other,other_idx).m_r;
		unsigned char g = GetKey(other,other_idx).m_g;
		unsigned char b = GetKey(other,other_idx).m_b;

		colors[pt_count] = v3_new((double) r,
					  (double) g,
					  (double) b);

		ImageKeyVector views;
		views.push_back(ImageKey(i, other_idx));
		views.push_back(ImageKey(camera_idx, this_idx));
		pt_views.push_back(views);

		pt_count++;

	    } else if (GetKey(other,other_idx).m_extra >= 0 &&
		       GetKey(image_idx,this_idx).m_extra == -1) {

		int pt_idx = GetKey(other,other_idx).m_extra;

		//检察重投影是否出错
		v2_t pr = sfm_project_final(cameras + camera_idx,
					    points[pt_idx],
					    true, m_estimate_distortion);

		double dx = GetKey(image_idx,this_idx).m_x - Vx(pr);
		double dy = GetKey(image_idx,this_idx).m_y - Vy(pr);

		double proj_error = sqrt(dx * dx + dy * dy);

		if (proj_error <= INIT_REPROJECTION_ERROR) {
		    printf("  Reconnecting point [%d] (%d) (error: %0.3f)\n",
			   pt_idx, this_idx, proj_error);
		    GetKey(image_idx,this_idx).m_extra = pt_idx;
		    pt_views[pt_idx].push_back(ImageKey(camera_idx,this_idx));
		} else {
		    //将这一点作为一个异常点抛出
		    GetKey(image_idx,this_idx).m_extra = -2;
		}
	    }
	}

        m_matches.ClearMatch(idx);
    }

    return pt_count;
}
