#ifndef __keys2a_h__
#define __keys2a_h__

#include <vector>
#include <stdio.h>

#include <zlib.h>

#include "ANN/ANN.h"

using namespace ann_1_1_char;

class Keypoint {
public:
    Keypoint(float x, float y, float scale, float ori, short int *d) :
	m_x(x), m_y(y), m_scale(scale), m_ori(ori), m_d(d)
    { }

    float m_x, m_y;        
    float m_scale, m_ori;  
    short int *m_d; 
};


/* 匹配信息的结构 */
class KeypointMatch {
public:
    KeypointMatch()
    { }


    KeypointMatch(int idx1, int idx2) :
	m_idx1(idx1), m_idx2(idx2)
    { }

    int m_idx1, m_idx2;
};

typedef struct {
    float x, y;
    float scale;
    float orient;
} keypt_t;

/* 返回文件中的特征点数量 */
int GetNumberOfKeys(const char *filename);

/* 通过文件名读取特征点 */
int ReadKeyFile(const char *filename, unsigned char **keys,
                keypt_t **info = NULL);

int ReadKeyPositions(const char *filename, keypt_t **info);

/* 通过文件指针读取特征点列表。文件格式为：第一个整数，表示该文件特征点
 * 数量，第二个整数，表示特征描述向量维度（默认为128）。接下来4个浮点类
 * 型数表示点的像素坐标x，y，尺度，旋转角度（-PI到PI）。接下来是每个特征
 * 点的128维描述向量，值为[0，255] */
int ReadKeys(FILE *fp, unsigned char **keys, keypt_t **info = NULL);
int ReadKeysGzip(gzFile fp, unsigned char **keys, keypt_t **info = NULL);

/* 通过MMAP加速特征点读取 */
std::vector<Keypoint *> ReadKeysMMAP(FILE *fp);

/* 对一组特征点建立搜索树 */
ANNkd_tree *CreateSearchTree(int num_keys, unsigned char *keys);

/* 计算两组特征点的相似度 */
std::vector<KeypointMatch> MatchKeys(int num_keys1, unsigned char *k1,
				     int num_keys2, unsigned char *k2,
				     double ratio = 0.6,
                                     int max_pts_visit = 200);

std::vector<KeypointMatch> MatchKeys(int num_keys1, unsigned char *k1,
                                     ANNkd_tree *tree2,
				     double ratio = 0.6,
                                     int max_pts_visit = 200);

#endif 
