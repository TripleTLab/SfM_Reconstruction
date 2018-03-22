#include "BaseApp.h"
#include "defines.h"

/* 获取图像特征点 */
Keypoint &BaseApp::GetKey(int img, int key) {
    return m_image_data[img].m_keys[key];
}

/* 获取图像特征点的描述向量 */
KeypointWithDesc &BaseApp::GetKeyWithDesc(int img, int key) {
    return m_image_data[img].m_keys_desc[key];
}

/* 获取图像特征点的数量 */
int BaseApp::GetNumKeys(int img) {
    return (int) m_image_data[img].m_keys.size();
}

/* 获取相机与对应图像的索引 */
int BaseApp::GetRegisteredCameraIndex(int cam) {
    int num_images = GetNumImages();

    int count = 0;
    for (int i = 0; i < num_images; i++) {
	if (m_image_data[i].m_camera.m_adjusted) {
	    if (count == cam)
		return i;

	    count++;
	}
    }

    printf("[GetRegisteredCameraIndex] "
	   "Error: ran out of cameras\n");

    return -1;
}

/* 返回图像数量 */
int BaseApp::GetNumImages() {
    return (int) m_image_data.size();
}

/* 返回原始图像数量 */
int BaseApp::GetNumOriginalImages() {
    return m_num_original_images;
}

/* 获取匹配数量 */
int BaseApp::GetNumMatches(int i1, int i2)
{
    int i_min = MIN(i1, i2);
    int i_max = MAX(i1, i2);

    MatchIndex idx = GetMatchIndex(i_min, i_max);

    return m_matches.GetNumMatches(idx);
}

#if 1
/* 获取匹配信息 */
MatchIndex GetMatchIndex(int i1, int i2) {
    return MatchIndex((unsigned long) i1, (unsigned long) i2);
}

/* 获取无序图像的匹配信息 */
MatchIndex GetMatchIndexUnordered(int i1, int i2) {
    if (i1 < i2)
        return MatchIndex((unsigned long) i1, (unsigned long) i2);
    else
        return MatchIndex((unsigned long) i2, (unsigned long) i1);
}
#endif

/* 设置匹配信息 */
void BaseApp::SetMatch(int i1, int i2) {
    m_matches.SetMatch(GetMatchIndex(i1, i2));
}


/* 移除匹配信息 */
void BaseApp::RemoveMatch(int i1, int i2) {
    m_matches.RemoveMatch(GetMatchIndex(i1, i2));
}

/* 判断图像是否匹配 */
bool BaseApp::ImagesMatch(int i1, int i2) {
    return m_matches.Contains(GetMatchIndex(i1, i2));
}

/* 根据图像名找到图像 */
int BaseApp::FindImageWithName(const char *name)
{
    int num_images = GetNumImages();

    for (int i = 0; i < num_images; i++) {
	if (strcmp(m_image_data[i].m_name, name) == 0)
	    return i;
    }

    return -1;
}

/* 重置调整后的匹配点的索引 */
void BaseApp::ReindexPoints()
{
    int num_images = GetNumImages();

    int adjusted = 0;
    int *reindex = new int[num_images];

    m_num_views_orig.clear();

    for (int i = 0; i < num_images; i++) {
        if (m_image_data[i].m_camera.m_adjusted &&
            m_image_data[i].m_licensed) {
            reindex[i] = adjusted;
            adjusted++;
        }
    }

    int num_points = m_point_data.size();

    for (int i = 0; i < num_points; i++) {
        int num_views = (int) m_point_data[i].m_views.size();
        m_num_views_orig.push_back(num_views);

        for (int j = 0; j < num_views; j++) {
            int v = m_point_data[i].m_views[j].first;

            if (!m_image_data[v].m_camera.m_adjusted ||
                !m_image_data[v].m_licensed) {
                m_point_data[i].m_views.
                    erase(m_point_data[i].m_views.begin() + j);
                j--;
                num_views--;
            } else {
                m_point_data[i].m_views[j].first = reindex[v];
            }
        }
    }

    delete [] reindex;
}
