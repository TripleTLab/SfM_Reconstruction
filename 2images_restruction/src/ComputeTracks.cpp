


#include <queue>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "keys.h"

#include "BundlerApp.h"
#include "BundleUtil.h"

bool CompareFirst(const KeypointMatch &k1, const KeypointMatch &k2) {
    return (k1.m_idx1 < k2.m_idx1);
}


#define LARGE_NUMBER 99999999
void BundlerApp::ComputeTracks(int new_image_start)
{
    unsigned int num_images = GetNumImages();

    
    for (unsigned int i = 0; i < num_images; i++) {
        
        // std::list<unsigned int> &nbrs = m_matches.GetNeighbors(i);
        // int num_nbrs = (int) nbrs.size();
        int num_nbrs = (int) m_matches.GetNumNeighbors(i);

        if (num_nbrs == 0)
            continue;

        // m_image_data[i].LoadKeys(false);
	// int num_features = GetNumKeys(i);
        int num_features = m_image_data[i].GetNumKeys();
        m_image_data[i].m_key_flags.resize(num_features);

	// for (int j = 0; j < num_features; j++) {
	//     GetKey(i,j).m_extra = -1;
	// }
    }

    int pt_idx = 0;

    std::vector<TrackData> tracks;

    // KeymatchHashHash hashes;
    // CreateKeymatchHashes(hashes);

    
#if 0
    for (int i = 0; i < num_images; i++) {
        for (int j = 0; j < num_images; j++) {
            if (i == j)
                continue;

            if (!ImagesMatch(i,j))
                continue;

            MatchIndex idx = GetMatchIndex(i, j);

            std::vector<KeypointMatch> &list = m_match_lists[idx];

            sort(list.begin(), list.end(), CompareFirst);
        }
    }
#else
    for (unsigned int i = 0; i < num_images; i++) {
        MatchAdjList::iterator iter;
        for (iter = m_matches.Begin(i); iter != m_matches.End(i); iter++) {
            // MatchIndex idx = *iter;
            std::vector<KeypointMatch> &list = iter->m_match_list; // iter->second; // m_match_lists[idx];
            sort(list.begin(), list.end(), CompareFirst);
        }
    }
#endif

    bool *img_marked = new bool[num_images];
    memset(img_marked, 0, num_images * sizeof(bool));

    std::vector<int> touched;
    touched.reserve(num_images);

    for (unsigned int i = 0; i < num_images; i++) {
	int num_features = m_image_data[i].GetNumKeys(); // GetNumKeys(i);

        
        // std::list<unsigned int> &nbrs = m_matches.GetNeighbors(i);
        // int num_nbrs = (int) nbrs.size();
        int num_nbrs = (int) m_matches.GetNumNeighbors(i);

        if (num_nbrs == 0)
            continue;

	for (int j = 0; j < num_features; j++) {
	    ImageKeyVector features;
	    std::queue<ImageKey> features_queue;

	    
	    // if (GetKey(i,j).m_extra >= 0)
            //      continue;
            if (m_image_data[i].m_key_flags[j])
                continue; // already visited this feature

            // memset(img_marked, 0, num_images * sizeof(bool));
            
            int num_touched = touched.size();
            for (int k = 0; k < num_touched; k++)
                img_marked[touched[k]] = false;
            touched.clear();

	    
	    // GetKey(i,j).m_extra = pt_idx;
            m_image_data[i].m_key_flags[j] = true;

	    features.push_back(ImageKey(i, j));
	    features_queue.push(ImageKey(i, j));

            img_marked[i] = true;
            touched.push_back(i);

	    int num_rounds = 0;
	    while (!features_queue.empty()) {
		num_rounds++;

		ImageKey feature = features_queue.front();
		features_queue.pop();

		int img1 = feature.first;
		int f1 = feature.second;
                KeypointMatch dummy;
                dummy.m_idx1 = f1;

		int start_idx;
		
		if (img1 >= new_image_start) {
		    start_idx = new_image_start;
		} else {
		    start_idx = 0;
		}

		// for (unsigned int k = start_idx; k < num_images; k++) {
                
                // std::list<unsigned int> &nbrs = m_matches.GetNeighbors(img1);
                // int num_nbrs = (int) nbrs.size();
                MatchAdjList &nbrs = m_matches.GetNeighbors(img1);

                // std::list<unsigned int>::iterator iter;
                MatchAdjList::iterator iter;
                for (iter = nbrs.begin(); iter != nbrs.end(); iter++) {
                    // for (int nbr = 0; nbr < num_nbrs; nbr++) {
                    unsigned int k = iter->m_index; // *iter; // nbrs[nbr];

                    if (img_marked[k])
                        continue;

		    
		    // if (!ImagesMatch(img1, k))
                    //     continue;

		    MatchIndex base = GetMatchIndex(img1, k);

                    std::vector<KeypointMatch> &list =
                        m_matches.GetMatchList(base); // m_match_lists[base];

                    
                    std::pair<std::vector<KeypointMatch>::iterator,
                              std::vector<KeypointMatch>::iterator> p;

                    p = equal_range(list.begin(), list.end(),
                                    dummy, CompareFirst);

                    if (p.first == p.second)
                        continue;  

                    assert((p.first)->m_idx1 == f1);
                    int idx2 = (p.first)->m_idx2;

                    
                    // if (GetKey(k,idx2).m_extra >= 0)
                    //     continue;
                    assert(idx2 < m_image_data[k].GetNumKeys());

                    if (m_image_data[k].m_key_flags[idx2])
                        continue;

                    
                    // GetKey(k,idx2).m_extra = pt_idx;
                    m_image_data[k].m_key_flags[idx2] = true;
                    features.push_back(ImageKey(k, idx2));
                    features_queue.push(ImageKey(k, idx2));

                    img_marked[k] = true;
                    touched.push_back(k);
		}
	    } 

#if 0
	    
	    bool consistent;
	    int num_inconsistent = 0;
	    do {
		consistent = true;

		for (int k = 0; k < num_images; k++) {
		    int track_size = (int) features.size();
		    int count = 0;
		    for (int l = 0; l < track_size; l++) {
			if (features[l].first == k)
			    count++;
		    }

		    if (count > 1) {
			/* Remove the points associated with this
			 * image */

			for (int l = 0; l < track_size; l++) {
			    if (features[l].first == k) {
				int img = features[l].first;
				int idx = features[l].second;

				// GetKey(img,idx).m_extra = LARGE_NUMBER;

				features.erase(features.begin() + l);
				track_size--;
				l--;
			    }
			}

			consistent = false;
			num_inconsistent++;

			break;
		    }
		}
	    } while (!consistent);
#endif

	    if (features.size() >= 2) {
		printf("Point with %d projections found\n",
                       //  (%d inconsistent)\n",
		       (int) features.size()); // , num_inconsistent);
		fflush(stdout);

		tracks.push_back(TrackData(features));

		pt_idx++;
	    } else {
		// printf("Feature only has %d points (%d inconsistent)\n",
		//       (int) features.size(), num_inconsistent);
	    }

	} 
    } 

    printf("[ComputeTracks] Found %d points\n", pt_idx);
    fflush(stdout);

    if (pt_idx != (int) tracks.size()) {
	printf("[ComputeTracks] Error: point count "
	       "inconsistent!\n");
	fflush(stdout);
    }

    
    printf("[ComputeTracks] Clearing match lists...\n");
    fflush(stdout);

    RemoveAllMatches();

    
    printf("[ComputeTracks] Creating consistent match lists...\n");
    fflush(stdout);

    int num_pts = pt_idx;

    for (int i = 0; i < num_pts; i++) {
	int num_features = (int) tracks[i].m_views.size();

        for (int j = 0; j < num_features; j++) {
            int img1 = tracks[i].m_views[j].first;
            int key1 = tracks[i].m_views[j].second;

            m_image_data[img1].m_visible_points.push_back(i);
            m_image_data[img1].m_visible_keys.push_back(key1);
        }
    }

    
    m_track_data = tracks;

    // SetMatchesFromTracks();

    printf("[ComputeTracks] Done!\n");
    fflush(stdout);
}
#undef LARGE_NUMBER
