#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "defines.h"
#include "matrix.h"
#include "triangulate.h"
#include "util.h"

#include "BundlerApp.h"
#include "Bundle.h"

#define MIN_INLIERS_EST_PROJECTION 15
#define INIT_REPROJECTION_ERROR 16.0 

/* 快速计算相机位姿 */
void BundlerApp::BundleAdjustFast()
{
    clock_t start = clock();

    /* 计算初始图像信息 */
    ComputeGeometricConstraints();


    /* 将轨迹指针设为 -1 */
    for (int i = 0; i < (int) m_track_data.size(); i++) {
	m_track_data[i].m_extra = -1;
    }

    /* 假设所有图像组合成一组 */
    int num_images = GetNumImages();
    int *added_order = new int[num_images];
    int *added_order_inv = new int[num_images];


    /* **** 运行光束平差法 **** */

    camera_params_t *cameras = new camera_params_t[num_images];
    int max_pts = (int) m_track_data.size();
    v3_t *points = new v3_t[max_pts];
    v3_t *colors = new v3_t[max_pts];
    std::vector<ImageKeyVector> pt_views;


    /* 初始化光速平差法 */
    int num_init_cams = 0;
    InitializeBundleAdjust(num_init_cams, added_order, added_order_inv,
			   cameras, points, colors, pt_views,
			   m_use_constraints);

    int i_best = -1, j_best = -1, max_matches = 0;
    double max_score = 0.0;
    int curr_num_cameras, curr_num_pts;
    int pt_count;

    if (num_init_cams == 0) {
	BundlePickInitialPair(i_best, j_best, true);

	added_order[0] = i_best;
	added_order[1] = j_best;

	printf("[BundleAdjust] Adjusting cameras "
	       "%d and %d (score = %0.3f)\n",
	       i_best, j_best, max_score);

	/* **** Set up the initial cameras **** */
	double init_focal_length_0 = 0.0, init_focal_length_1 = 0.0;
	pt_count = curr_num_pts =
	    SetupInitialCameraPair(i_best, j_best,
				   init_focal_length_0, init_focal_length_1,
				   cameras, points, colors, pt_views);

        DumpOutputFile(m_output_directory, "bundle.init.out",
                       num_images, 2, curr_num_pts,
                       added_order, cameras, points, colors, pt_views);

	/* 第一次运行运动结构恢复 */
	double error0;
	error0 = RunSFM(curr_num_pts, 2, 0, false,
			cameras, points, added_order, colors, pt_views);

	printf("  focal lengths: %0.3f, %0.3f\n", cameras[0].f, cameras[1].f);

	if (m_fix_necker) {
	    
	    camera_params_t cameras_old[2];
	    v3_t *points_old;

	    points_old = new v3_t [curr_num_pts];

	    memcpy(points_old, points, sizeof(v3_t) * curr_num_pts);
	    memcpy(cameras_old, cameras, sizeof(camera_params_t) * 2);

	    camera_params_t tmp = cameras[0];
	    memcpy(cameras[0].R, cameras[1].R, sizeof(double) * 9);
	    memcpy(cameras[0].t, cameras[1].t, sizeof(double) * 3);
	    cameras[0].f = init_focal_length_0;
	    cameras[0].k[0] = cameras[0].k[1] = 0.0;

	    memcpy(cameras[1].R, tmp.R, sizeof(double) * 9);
	    memcpy(cameras[1].t, tmp.t, sizeof(double) * 3);
	    cameras[1].f = init_focal_length_1;
	    cameras[1].k[0] = cameras[1].k[1] = 0.0;

	    double K1inv[9] =
		{ 1.0 / cameras[0].f, 0.0, 0.0,
		  0.0, 1.0 / cameras[0].f, 0.0,
		  0.0, 0.0, 1.0 };

	    double K2inv[9] =
		{ 1.0 / cameras[1].f, 0.0, 0.0,
		  0.0, 1.0 / cameras[1].f, 0.0,
		  0.0, 0.0, 1.0 };

	    for (int i = 0; i < curr_num_pts; i++) {
		int k1 = pt_views[i][0].second;
		int k2 = pt_views[i][1].second;

		double proj1[3] = { GetKey(added_order[0],k1).m_x,
				    GetKey(added_order[0],k1).m_y,
				    -1.0 };

		double proj2[3] = { GetKey(added_order[1],k2).m_x,
				    GetKey(added_order[1],k2).m_y,
				    -1.0 };

		double proj1_norm[3], proj2_norm[3];

		matrix_product(3, 3, 3, 1, K1inv, proj1, proj1_norm);
		matrix_product(3, 3, 3, 1, K2inv, proj2, proj2_norm);

		v2_t p = v2_new(proj1_norm[0] / proj1_norm[2],
				proj1_norm[1] / proj1_norm[2]);

		v2_t q = v2_new(proj2_norm[0] / proj2_norm[2],
				proj2_norm[1] / proj2_norm[2]);

		double proj_error;

                double t1[3];
                double t2[3];

                /* 将转换设置为标准格式 */
                matrix_product(3, 3, 3, 1, cameras[0].R, cameras[0].t, t1);
                matrix_scale(3, 1, t1, -1.0, t1);
                matrix_product(3, 3, 3, 1, cameras[1].R, cameras[1].t, t2);
                matrix_scale(3, 1, t2, -1.0, t2);

                points[i] = triangulate(p, q,
                                        cameras[0].R, t1, cameras[1].R, t2,
                                        &proj_error);
	    }

	    double error1;
	    error1 = RunSFM(curr_num_pts, 2, 0, false,
			    cameras, points, added_order, colors, pt_views);

	}

	DumpPointsToPly(m_output_directory, "points001.ply",
                        curr_num_pts, 2, points, colors, cameras);

	if (m_bundle_output_base != NULL) {
	    char buf[256];
	    sprintf(buf, "%s%03d.out", m_bundle_output_base, 1);
	    DumpOutputFile(m_output_directory, buf, num_images, 2, curr_num_pts,
			   added_order, cameras, points, colors, pt_views);

	}

	curr_num_cameras = 2;
    } else {

	curr_num_cameras = num_init_cams;
	pt_count = curr_num_pts = (int) m_point_data.size();
    }

    int round = 0;
    while (curr_num_cameras < num_images) {
	int parent_idx;
	int max_cam =
            FindCameraWithMostMatches(curr_num_cameras, curr_num_pts,
                                      added_order, parent_idx,
                                      max_matches, pt_views);

	printf("[BundleAdjust] max_matches = %d\n", max_matches);

	if (max_matches < m_min_max_matches)
	    break; 
	/* 找到所有匹配数量大于75%的图像 */
	std::vector<ImagePair> image_set;

        if (false && max_matches < 48) { 
            image_set.push_back(ImagePair(max_cam, parent_idx));
        } else {
            int nMatches = iround(0.75 * max_matches);

            if (m_num_matches_add_camera > 0) {
                
                nMatches = std::min(nMatches, m_num_matches_add_camera);
            }

	    image_set =
                FindCamerasWithNMatches(nMatches,
                                        curr_num_cameras, curr_num_pts,
                                        added_order, pt_views);
        }

	int num_added_images = (int) image_set.size();

	printf("[BundleAdjustFast] Registering %d images\n",
	       num_added_images);

	for (int i = 0; i < num_added_images; i++)
	    printf("[BundleAdjustFast] Adjusting camera %d\n",
		   image_set[i].first);

	/* 将新相机混合进去 */
        int image_count = 0;
	for (int i = 0; i < num_added_images; i++) {
	    int next_idx = image_set[i].first;
	    int parent_idx = image_set[i].second;

	    added_order[curr_num_cameras + image_count] = next_idx;

	    printf("[BundleAdjust[%d]] Adjusting camera %d "
		   "(parent = %d)\n",
		   round, next_idx,
                   (parent_idx == -1 ? -1 : added_order[parent_idx]));

	    /* **** 初始化新相机 **** */
            bool success = false;
            camera_params_t camera_new =
		BundleInitializeImage(m_image_data[next_idx],
				      next_idx,
                                      curr_num_cameras + image_count,
				      curr_num_cameras, curr_num_pts,
				      added_order, points,
				      NULL, cameras,
				      pt_views, &success);

            if (success) {
                cameras[curr_num_cameras+image_count] = camera_new;
                image_count++;
            } else {
                printf("[BundleAdjust] Couldn't initialize image %d\n",
                       next_idx);
                m_image_data[next_idx].m_ignore_in_bundle = true;
            }
	}


#if 0
	double dist0 = GetCameraDistance(cameras + good_pair_1,
					 cameras + good_pair_2,
					 m_explicit_camera_centers);
#else
        double dist0 = 0.0;
#endif

	printf("[BundleAdjust] Adding new matches\n");

	pt_count = curr_num_pts;

	curr_num_cameras += image_count;

        if (!m_skip_add_points) {
            pt_count =
                BundleAdjustAddAllNewPoints(pt_count, curr_num_cameras,
                                            added_order, cameras,
                                            points, colors,
                                            dist0, pt_views);
        }

	curr_num_pts = pt_count;

	printf("[BundleAdjust] Number of points = %d\n", pt_count);
	fflush(stdout);

        if (!m_skip_full_bundle) {
            /* 第二次运行运动结构恢复来更新参数 */
            RunSFM(curr_num_pts, curr_num_cameras, 0, false,
                   cameras, points, added_order, colors, pt_views);

            /* 移除不好的点和相机 */
            RemoveBadPointsAndCameras(curr_num_pts, curr_num_cameras + 1,
                                      added_order, cameras, points, colors,
                                      pt_views);

            printf("  focal lengths:\n");

            for (int i = 0; i < curr_num_cameras; i++) {
                if(m_image_data[added_order[i]].m_has_init_focal) {
                    printf("   [%03d] %0.3f (%0.3f) %s %d; %0.3e %0.3e\n",
                           i, cameras[i].f,
                           m_image_data[added_order[i]].m_init_focal,
                           m_image_data[added_order[i]].m_name,
                           added_order[i], cameras[i].k[0], cameras[i].k[1]);
                } else {
                    printf("   [%03d] %0.3f %s %d; %0.3e %0.3e\n",
                           i, cameras[i].f,
                           m_image_data[added_order[i]].m_name,
                           added_order[i], cameras[i].k[0], cameras[i].k[1]);
                }
            }

            fflush(stdout);
        }


	/* 将该轮信息输出到文件 */
	char buf[256];
	sprintf(buf, "points%03d.ply", curr_num_cameras);

	DumpPointsToPly(m_output_directory, buf,
                        curr_num_pts, curr_num_cameras,
                        points, colors, cameras);

	if (m_bundle_output_base != NULL) {
	    sprintf(buf, "%s%03d.out", m_bundle_output_base,
                    curr_num_cameras);
	    DumpOutputFile(m_output_directory, buf,
                           num_images, curr_num_cameras, curr_num_pts,
			   added_order, cameras, points, colors, pt_views);
	}

	round++;
    }

    clock_t end = clock();

    printf("[BundleAdjust] Bundle adjustment took %0.3fs\n",
	   (end - start) / ((double) CLOCKS_PER_SEC));

    if (m_estimate_ignored) {
        EstimateIgnoredCameras(curr_num_cameras,
                               cameras, added_order,
                               curr_num_pts, points, colors, pt_views);
    }

    /* 输出文件 */
    if (m_bundle_output_file != NULL) {
	DumpOutputFile(m_output_directory, m_bundle_output_file,
		       num_images, curr_num_cameras, curr_num_pts,
		       added_order, cameras, points, colors, pt_views);
    }

    /* 保存相机参数和点 */

    /* 相机 */
    for (int i = 0; i < num_images; i++) {
	m_image_data[i].m_camera.m_adjusted = false;
    }

    for (int i = 0; i < curr_num_cameras; i++) {
	int img = added_order[i];

	m_image_data[img].m_camera.m_adjusted = true;
	memcpy(m_image_data[img].m_camera.m_R, cameras[i].R,
	       9 * sizeof(double));

        matrix_product(3, 3, 3, 1,
                       cameras[i].R, cameras[i].t,
                       m_image_data[img].m_camera.m_t);

        matrix_scale(3, 1,
                     m_image_data[img].m_camera.m_t, -1.0,
                     m_image_data[img].m_camera.m_t);

	m_image_data[img].m_camera.m_focal = cameras[i].f;

	m_image_data[img].m_camera.Finalize();
    }

    /* 点 */
    for (int i = 0; i < curr_num_pts; i++) {
	/* 检查该点是否在某个视角中可见 */
	if ((int) pt_views[i].size() == 0)
	    continue; 

	PointData pdata;
	pdata.m_pos[0] = Vx(points[i]);
	pdata.m_pos[1] = Vy(points[i]);
	pdata.m_pos[2] = Vz(points[i]);

	pdata.m_color[0] = (float) Vx(colors[i]);
	pdata.m_color[1] = (float) Vy(colors[i]);
	pdata.m_color[2] = (float) Vz(colors[i]);

#if 1
	for (int j = 0; j < (int) pt_views[i].size(); j++) {
	    int v = pt_views[i][j].first;
	    int vnew = added_order[v];
	    pdata.m_views.push_back(ImageKey(vnew, pt_views[i][j].second));
	}
#else

#endif

	m_point_data.push_back(pdata);
    }

    delete [] added_order;
    delete [] added_order_inv;

    SetMatchesFromPoints();

    bool *image_mask = new bool[num_images];

    for (int i = 0; i < num_images; i++) {
	if (m_image_data[i].m_camera.m_adjusted)
	    image_mask[i] = true;
	else
	    image_mask[i] = false;
    }
}
