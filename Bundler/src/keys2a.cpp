#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <zlib.h>

#include "keys2a.h"

int GetNumberOfKeysNormal(FILE *fp)
{
    int num, len;

    if (fscanf(fp, "%d %d", &num, &len) != 2) {
        printf("Invalid keypoint file.\n");
        return 0;
    }

    return num;
}

int GetNumberOfKeysGzip(gzFile fp)
{
    int num, len;

    char header[256];
    gzgets(fp, header, 256);

    if (sscanf(header, "%d %d", &num, &len) != 2) {
        printf("Invalid keypoint file.\n");
        return 0;
    }

    return num;
}

/* 返回文件中的特征点数量 */
int GetNumberOfKeys(const char *filename)
{
    FILE *file;

    file = fopen (filename, "r");
    if (! file) {
        char buf[1024];
        sprintf(buf, "%s.gz", filename);
        gzFile gzf = gzopen(buf, "rb");

        if (gzf == NULL) {
            printf("Could not open file: %s\n", filename);
            return 0;
        } else {
            int n = GetNumberOfKeysGzip(gzf);
            gzclose(gzf);
            return n;
        }
    }

    int n = GetNumberOfKeysNormal(file);
    fclose(file);
    return n;
}

/* 通过文件名读取特征点，返回特征点列表 */
int ReadKeyFile(const char *filename, unsigned char **keys, keypt_t **info)
{
    FILE *file;

    file = fopen (filename, "r");
    if (! file) {
        char buf[1024];
        sprintf(buf, "%s.gz", filename);
        gzFile gzf = gzopen(buf, "rb");

        if (gzf == NULL) {
            printf("Could not open file: %s\n", filename);
            return 0;
        } else {
            int n = ReadKeysGzip(gzf, keys, info);
            gzclose(gzf);
            return n;
        }
    }

    int n = ReadKeys(file, keys, info);
    fclose(file);
    return n;

}

/* 通过文件指针读取特征点列表。文件格式为：第一个整数，表示该文件特征点
 * 数量，第二个整数，表示特征描述向量维度（默认为128）。接下来4个浮点类
 * 型数表示点的像素坐标x，y，尺度，旋转角度（-PI到PI）。接下来是每个特征
 * 点的128维描述向量，值为[0，255] */
int ReadKeys(FILE *fp, unsigned char **keys, keypt_t **info)
{
    int i, num, len;

    std::vector<Keypoint *> kps;

    if (fscanf(fp, "%d %d", &num, &len) != 2) {
        printf("Invalid keypoint file\n");
        return 0;
    }

    if (len != 128) {
        printf("Keypoint descriptor length invalid (should be 128).");
        return 0;
    }

    *keys = new unsigned char[128 * num + 8];

    if (info != NULL)
        *info = new keypt_t[num];

    unsigned char *p = *keys;
    for (i = 0; i < num; i++) {
        float x, y, scale, ori;

        if (fscanf(fp, "%f %f %f %f\n", &y, &x, &scale, &ori) != 4) {
            printf("Invalid keypoint file format.");
            return 0;
        }

        if (info != NULL) {
            (*info)[i].x = x;
            (*info)[i].y = y;
            (*info)[i].scale = scale;
            (*info)[i].orient = ori;
        }

        char buf[1024];
        for (int line = 0; line < 7; line++) {
            fgets(buf, 1024, fp);

            if (line < 6) {
                sscanf(buf,
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu "
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                    p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7, p+8, p+9,
                    p+10, p+11, p+12, p+13, p+14,
                    p+15, p+16, p+17, p+18, p+19);

                p += 20;
            } else {
                sscanf(buf,
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                    p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7);
                p += 8;
            }
        }
    }

    return num;
}

int ReadKeysGzip(gzFile fp, unsigned char **keys, keypt_t **info)
{
    int i, num, len;

    std::vector<Keypoint *> kps;
    char header[256];
    gzgets(fp, header, 256);

    if (sscanf(header, "%d %d", &num, &len) != 2) {
        printf("Invalid keypoint file.\n");
        return 0;
    }

    if (len != 128) {
        printf("Keypoint descriptor length invalid (should be 128).");
        return 0;
    }

    *keys = new unsigned char[128 * num + 8];

    if (info != NULL)
        *info = new keypt_t[num];

    unsigned char *p = *keys;
    for (i = 0; i < num; i++) {
        float x, y, scale, ori;
        char buf[1024];
        gzgets(fp, buf, 1024);

        if (sscanf(buf, "%f %f %f %f\n", &y, &x, &scale, &ori) != 4) {
            printf("Invalid keypoint file format.");
            return 0;
        }

        if (info != NULL) {
            (*info)[i].x = x;
            (*info)[i].y = y;
            (*info)[i].scale = scale;
            (*info)[i].orient = ori;
        }

        for (int line = 0; line < 7; line++) {
            char *str = gzgets(fp, buf, 1024);
            assert(str != Z_NULL);

            if (line < 6) {
                sscanf(buf,
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu "
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                    p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7, p+8, p+9,
                    p+10, p+11, p+12, p+13, p+14,
                    p+15, p+16, p+17, p+18, p+19);

                p += 20;
            } else {
                sscanf(buf,
                    "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
                    p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7);
                p += 8;
            }
        }
    }

    assert(p == *keys + 128 * num);

    return num; 
}

/* 建立搜索树 */
ANNkd_tree *CreateSearchTree(int num_keys, unsigned char *keys)
{
    ANNpointArray pts = annAllocPts(num_keys, 128);

    for (int i = 0; i < num_keys; i++) {
        memcpy(pts[i], keys + 128 * i, sizeof(unsigned char) * 128);
    }

    ANNkd_tree *tree = new ANNkd_tree(pts, num_keys, 128, 16);

    return tree;
}

std::vector<KeypointMatch> MatchKeys(int num_keys1, unsigned char *k1,
                                     ANNkd_tree *tree2,
                                     double ratio, int max_pts_visit)
{
    annMaxPtsVisit(max_pts_visit);
    std::vector<KeypointMatch> matches;

    for (int i = 0; i < num_keys1; i++) {
        ANNidx nn_idx[2];
        ANNdist dist[2];

        tree2->annkPriSearch(k1 + 128 * i, 2, nn_idx, dist, 0.0);

        if (((double) dist[0]) < ratio * ratio * ((double) dist[1])) {
            matches.push_back(KeypointMatch(i, nn_idx[0]));
        }
    }

    return matches;
}

/* 计算两组特征点的相似度 */
std::vector<KeypointMatch> MatchKeys(int num_keys1, unsigned char *k1,
                                     int num_keys2, unsigned char *k2,
                                     double ratio, int max_pts_visit)
{
    annMaxPtsVisit(max_pts_visit);

    int num_pts = 0;
    std::vector<KeypointMatch> matches;

    num_pts = num_keys2;
    clock_t start = clock();

    ANNpointArray pts = annAllocPts(num_pts, 128);

    for (int i = 0; i < num_pts; i++) {
        memcpy(pts[i], k2 + 128 * i, sizeof(unsigned char) * 128);
    }

    ANNkd_tree *tree = new ANNkd_tree(pts, num_pts, 128, 16);
    clock_t end = clock();

    start = clock();
    for (int i = 0; i < num_keys1; i++) {
        ANNidx nn_idx[2];
        ANNdist dist[2];

        tree->annkPriSearch(k1 + 128 * i, 2, nn_idx, dist, 0.0);

        if (((double) dist[0]) < ratio * ratio * ((double) dist[1])) {
            matches.push_back(KeypointMatch(i, nn_idx[0]));
        }
    }
    end = clock();
    annDeallocPts(pts);

    delete tree;

    return matches;
}
