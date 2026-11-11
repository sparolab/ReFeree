#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <yeti/CurrReferee.h>

#define SIZE 50
using std::cout;
using std::endl;

float LOOP_THRES = 0.9;
const int NUM_EXCLUDE_RECENT = 30;

// Base Class of ReFeree Descriptor
class ReFeree {
    public:
        ReFeree(): dim_(0) {}
        ReFeree(int dim): dim_(dim) {}
        ReFeree(std::vector<float> referee): desc(referee), dim_(referee.size()) {}
        virtual void makeDescriptor(Eigen::MatrixXd polar) {}
        void push_back(float d) { 
            if(desc.size() < dim_) desc.push_back(d);
            else std::cout << "Unable to edit ReFeree." << std::endl;
        }
        double norm();
        int size() { return desc.size(); }
        void clear() { desc.clear(); }
        bool empty() { return desc.empty(); }
        friend std::ostream& operator<<(std::ostream& os, ReFeree const& referee) {
            for(auto d: referee.desc) os << d << " ";
            return os;
        }

        std::vector<float> desc;
    protected:
        int dim_;
};

// Base Class of Range-ReFeree Descriptor
class ReFereeR: public ReFeree {
    public:
        ReFereeR(): ReFeree() {}
        ReFereeR(Eigen::MatrixXd polar): ReFeree() { makeDescriptor(polar); }
        ReFereeR(Eigen::MatrixXd polar, int dim): ReFeree(dim) { makeDescriptor(polar); }
        ReFereeR(std::vector<float> referee): ReFeree(referee) {}
        void makeDescriptor(Eigen::MatrixXd polar) override;
        double score(ReFereeR other);
};

// Base Class of Angle-ReFeree Descriptor
class ReFereeA: public ReFeree {
    public:
        ReFereeA(): ReFeree(SIZE) {}
        ReFereeA(Eigen::MatrixXd polar): ReFeree(SIZE) { makeDescriptor(polar); }
        ReFereeA(Eigen::MatrixXd polar, int dim): ReFeree(dim) { makeDescriptor(polar); }
        ReFereeA(std::vector<float> referee): ReFeree(referee) {}
        void makeDescriptor(Eigen::MatrixXd polar) override;
        // float score(ReFereeA other);
        int maxScoreIdx(ReFereeA query_desc, std::vector<std::vector<int>>* ids);
};

class ReFereeDB {
    public:
        ReFereeDB(): desc_dim_(0) {}
        ReFereeDB(int desc_dim): desc_dim_(desc_dim) {}
        void setDescSize(int dim) { if(desc_dim_ == 0) desc_dim_ = dim; }
    protected:
        int desc_dim_;
};

class ReFereeRDB: public ReFereeDB {
    public:
        ReFereeRDB() {}
        ReFereeRDB(int desc_dim): ReFereeDB(desc_dim) {}
        int detectLoopClosureID(ReFereeR curr_refereeR);
        ReFereeR getDesc(int idx) { return db_[idx]; }
        void push_back(ReFereeR desc) { db_.push_back(desc); }
        int size() { return db_.size(); }
        bool empty() { return db_.empty(); }
    private:
        std::vector<ReFereeR> db_;
};

class ReFereeADB: public ReFereeDB {
    public:
        ReFereeADB(): ReFereeDB(SIZE) { setIDS(); }
        ReFereeADB(int desc_dim): ReFereeDB(desc_dim) { setIDS(); }
        ReFereeA getDesc(int idx) { return db_[idx]; }
        float getYawDiff(ReFereeA query_desc, int loop_id);
        void push_back(ReFereeA desc) { db_.push_back(desc); }
        int size() { return db_.size(); }
        bool empty() { return db_.empty(); }
    private:
        void setIDS();
        std::vector<ReFereeA> db_;
        std::vector<std::vector<int>> ids_;
};

ReFereeR curr_refereeR;
ReFereeRDB refereeR_db;
ReFereeA curr_refereeA;
ReFereeADB refereeA_db;