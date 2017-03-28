//
// Created by eugen on 24.03.17.
//

#ifndef NNSEARCH_INCLUDED_H
#define NNSEARCH_INCLUDED_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <cstdlib>

#include <flann/flann.hpp>
template <typename T>
class NNSearch
{
public:
    
    flann::Index<flann::L2<T> >* index;
    T* data_ptr;

    NNSearch(){}
    ~NNSearch(){
        delete[] data_ptr;
        delete index;
    }

    /*Builds a 1D dataset using only timestamps*/
    void buildDatasetFromTimeStamps(std::vector<T>& data)
    {
        data_ptr= (T*) malloc (sizeof(T)*data.size()); //new T [data.size()];
        for (int i=0; i<data.size(); i++)
        {
            data_ptr[i]=data[i];
            //dataset(i,1)=data[i].second;
        }
        flann::Matrix<T> data_mat(data_ptr, data.size(), 1);
        index=new flann::Index<flann::L2<T> >(data_mat, flann::KDTreeIndexParams(4)); //flann::KDTreeSingleIndexParams(4));
        index->buildIndex();
    }

    /** returns the indices of the data used to build the dataset from time stamps
    */
    std::vector<int> search_by_time(T t, int nn=1)
    {
        flann::Matrix<T> qry(new T[1], 1, nn);
        *qry[0]=t;//access the first row pointer
        flann::Matrix<int> indices(new int[qry.rows*nn], qry.rows, nn);
        flann::Matrix<T> dists(new T[qry.rows*nn], qry.rows, nn);

        index->knnSearch(qry, indices, dists, nn, flann::SearchParams(128));
        std::vector<int> res;
        for (int i=0; i<nn; i++)
        {
            res.push_back(*(indices[0]+i));
        }
        return res;
    }

};
        


#endif //NNSEARCH_INCLUDED_H
