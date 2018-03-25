#ifndef __keys_h__
#define __keys_h__

#include <vector>
#include <stdio.h>
#include <zlib.h>

#ifndef __DEMO__
#ifdef __BUNDLER_DISTR__
#include "ANN/ANN.h"
#else
#include "ann_1.1_char/include/ANN/ANN.h"
#endif
#endif

class Keypoint {
public:
    Keypoint()
    { m_x = 0.0; m_y = 0.0; m_extra = -1; m_track = -1;
        m_r = m_g = m_b = 0; }

    Keypoint(float x, float y) :
	m_x(x), m_y(y)
    { m_extra = -1; m_track = -1; m_r = 0; m_g = 0; m_b = 0; }

    virtual ~Keypoint() {}

    virtual unsigned char *GetDesc() {
        return NULL;
    }

    float m_x, m_y;             
    unsigned char m_r, m_g, m_b; 

    int m_extra;
    int m_track;
};

class KeypointWithDesc : public Keypoint
{
public:
    virtual ~KeypointWithDesc() {}

    virtual unsigned char *GetDesc() {
        return m_d;
    }

    KeypointWithDesc()
    { m_x = 0.0; m_y = 0.0; m_d = NULL;
        m_extra = -1; m_track = -1; }

    KeypointWithDesc(float x, float y, unsigned char *d) :
	Keypoint(x, y), m_d(d)
    { }

    unsigned char *m_d;
};

class KeypointWithScaleRot : public KeypointWithDesc
{
public:
    virtual ~KeypointWithScaleRot() {}

    float m_scale, m_orient;
};

class KeypointMatch {
public:
    KeypointMatch()
    { }

#ifdef KEY_LIMIT
    KeypointMatch(unsigned short idx1, unsigned short idx2) :
#else
    KeypointMatch(int idx1, int idx2) :
#endif
	m_idx1(idx1), m_idx2(idx2)
    { }

#ifdef KEY_LIMIT
    unsigned short m_idx1, m_idx2;
#else
    int m_idx1, m_idx2;
#endif
};

class KeypointMatchWithScore : public KeypointMatch {
public:
    KeypointMatchWithScore()
    { }

#ifdef KEY_LIMIT
    KeypointMatchWithScore (unsigned short idx1, unsigned short idx2,
                            float score) :
#else
    KeypointMatchWithScore (int idx1, int idx2, float score) :
#endif
	KeypointMatch(idx1, idx2), m_score(score)
    { }

#ifdef KEY_LIMIT
    unsigned short m_idx1, m_idx2;
#else
    int m_idx1, m_idx2;
#endif

    float m_score;
};

typedef struct {
    float x, y;
    float scale;
    float orient;
} keypt_t;

/* 返回文件中特征点数量 */
int GetNumberOfKeys(const char *filename);

/* 通过文件名读取特征点，返回特征点列表 */
std::vector<Keypoint> ReadKeyFile(const char *filename);
std::vector<KeypointWithDesc>
    ReadKeyFileWithDesc(const char *filename, bool descriptor);
std::vector<KeypointWithScaleRot>
    ReadKeyFileWithScaleRot(const char *filename, bool descriptor);

/* 通过文件指针读取特征点，返回特征点列表 */
std::vector<Keypoint> ReadKeys(FILE *fp, bool descriptor);

/* 快速读取特征点 */
std::vector<KeypointWithDesc> ReadKeysFast(FILE *fp, bool descriptor,
                                           float **scale = NULL,
                                           float **orient = NULL);
std::vector<KeypointWithDesc> ReadKeysFastGzip(gzFile fp, bool descriptor,
                                               float **scale = NULL,
                                               float **orient = NULL);

/* 通过二进制文件读取特征点 */
std::vector<KeypointWithDesc> ReadKeysFastBin(FILE *fp, bool descriptor,
                                                 float **scales = NULL,
                                                 float **orients = NULL);

/* 通过gzip压缩的二进制文件读取特征点 */
std::vector<KeypointWithDesc> ReadKeysFastBinGzip(gzFile fp, bool descriptor,
                                                  float **scales = NULL,
                                                  float **orients = NULL);

#ifndef __DEMO__
/* 对对应特征点集建立搜索树 */
ann_1_1_char::ANNkd_tree
    *CreateSearchTreeChar(const std::vector<KeypointWithDesc> &k);
#endif 

/* 计算两组特征点的相似度 */
std::vector<KeypointMatch> MatchKeys(const std::vector<KeypointWithDesc> &k1,
				     const std::vector<KeypointWithDesc> &k2,
				     bool registered = false,
				     double ratio = 0.6);

/* 计算两组特征点的相似度 */
std::vector<KeypointMatchWithScore>
    MatchKeysWithScore(const std::vector<KeypointWithDesc> &k1,
                       const std::vector<KeypointWithDesc> &k2,
                       bool registered = false,
                       double ratio = 0.6);

/* 移除多余的匹配信息 */
std::vector<KeypointMatchWithScore>
    PruneMatchesWithScore(const std::vector<KeypointMatchWithScore> &matches);

std::vector<KeypointMatch>
    MatchKeysExhaustive(const std::vector<KeypointWithDesc> &k1,
                        const std::vector<KeypointWithDesc> &k2,
                        bool registered = false,
                        double ratio = 0.6);

#endif
