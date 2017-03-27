//
// Created by eugen on 24.03.17.
//

#ifndef NNSEARCH_INCLUDED_H
#define NNSEARCH_INCLUDED_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <flann/flann.hpp>

namespace gtsam{
    class NNSearch
    {
    public:
        
        flann::Index<flann::L2<uint64_t> > index;

        NNSearch(){
            int nn = 3;
            
            flann::Matrix<uint64_t> query;

            flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
            flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
            // construct an randomized kd-tree index using 4 kd-trees
            
            // do a knn search, using 128 checks
            index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
        };

        /*Builds a 1D dataset using only timestamps*/
        void buildDatasetFromTimeStamps(std::vector<uint64_t>& data)
        {
            flann::Matrix<uint64_t> dataset(data.size(), 1);
            for (int i=0; i<data.size(); i++)
            {
                dataset(i,0)=data[i];
                //dataset(i,1)=data[i].second;
            }


            index=flann::Index<flann::L2<uint64_t> >(dataset, flann::KDTreeSingleIndexParams(4));
            index.buildIndex();
        }

        /** returns the indices of the data used to build the dataset from time stamps
        */
        std::vector<uint64_t> search_by_time_nn(uint64_t t, int nn=1)
        {
            flann:Matrix<uint64_t> qry(1);
            qry(0,0)=t;
            flann::Matrix<int> indices;
            flann::Matrix<int> dists;
            index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
            std::vector<uint64_t> res;
            for (int i=0; i<nn; i++)
            {
                res.push_back(indices(i,0));
            }
            return res;
        }

    }
        
}


#endif //NNSEARCH_INCLUDED_H
