#include <vector>
#include <iostream>

#include "referee.hpp"

double ReFeree::norm() {
    double sum = 0.0;
    for(auto d: desc) {
        sum += pow(d, 2);
    }
    return sqrt(sum);
}

void ReFereeR::makeDescriptor(Eigen::MatrixXd polar) {
    Eigen::MatrixXd freespace_key(polar.cols(), 1);

    int window_size = 80;
    double freespace_counting = 0;
    int new_size = freespace_key.rows() / window_size;

    Eigen::MatrixXd freespace_result(new_size, 1);

    for (int col_idx = 0; col_idx < polar.cols(); col_idx++) {
        Eigen::MatrixXd curr_row = polar.col(col_idx);
        freespace_key(col_idx, 0) = polar.rows() - curr_row.sum();
    }

    for (int row_idx = 0; row_idx < freespace_key.rows(); row_idx++) {
        freespace_counting += freespace_key(row_idx);
        if (row_idx % window_size == 0) {
            int x = row_idx / window_size;
            freespace_result(x, 0) = freespace_counting;
            freespace_counting = 0;
        }
    }

    Eigen::MatrixXd referee = freespace_result;

    desc.resize(referee.size());
    for(int i = 0; i < referee.size(); ++i) {
        desc[i] = static_cast<float>(referee(i));
    }
}

double ReFereeR::score(ReFereeR other) {
    if (desc.size() != other.desc.size() || desc.empty() || other.desc.empty()) {
        return 0.0;
    }

    double dotProduct = 0.0;
    double mag1 = 0.0;
    double mag2 = 0.0;

    for (size_t i = 0; i < desc.size(); ++i) {
        dotProduct += desc[i] * other.desc[i];
        mag1 += desc[i] * desc[i];
        mag2 += other.desc[i] * other.desc[i];
    }

    mag1 = sqrt(mag1);
    mag2 = sqrt(mag2);

    if (mag1 == 0.0 || mag2 == 0.0) {
        return 0.0;
    }

    return dotProduct / (mag1 * mag2);
}

int ReFereeRDB::detectLoopClosureID(ReFereeR curr_refereeR) {
    int loop_id = -1;
    if(curr_refereeR.empty() && db_.empty()) return -1;

    if(db_.size() < NUM_EXCLUDE_RECENT + 1) return -1;

    double max_score = std::numeric_limits<double>::min();
    int nn_align = 0;
    int nn_idx = -1;
    for(size_t i = 0; i < db_.size() - NUM_EXCLUDE_RECENT; ++i) {
        auto referee_candidate = db_[i];
        double dist_result = curr_refereeR.score(referee_candidate);

        double candidate_dist = dist_result;
        if(candidate_dist > max_score) { // similarity distance thres
            max_score = candidate_dist;
            nn_idx = i;
        }
    }

    if(max_score > LOOP_THRES) { // gt threshold
        std::cout << "\033[1;32m[Loop found] Nearest distance: " << max_score << " between " << db_.size() - 1 << " and " << nn_idx << "\033[0m" << std::endl;
        loop_id = nn_idx;
    } else {
        std::cout << "\033[1;31m[Not loop] Nearest distance: " << max_score << " between " << db_.size() - 1 << " and " << nn_idx << "\033[0m" << std::endl;
    }

    return loop_id;
}

void ReFereeA::makeDescriptor(Eigen::MatrixXd polar) {
    desc.resize(dim_);
    int rbh = polar.rows() / dim_; // 8
    int rbw = polar.cols();
    int total_pix = rbh*rbw;
    for(int i = 0; i < dim_; i++) {
        
        float free_space_counter = 0.;
        for(int j = 0; j < rbh; j++) {
            Eigen::MatrixXd ray = polar.row(i * rbh + j);
            bool farthest_range = false;
            for(int k = rbw-1; k > 0; k--) {
                if(!farthest_range && ray(k) != 0.0) farthest_range = true;
                if(farthest_range) free_space_counter++;
            }
        }
        desc[i] = (float)free_space_counter / (float)total_pix;
    }
}

int ReFereeA::maxScoreIdx(ReFereeA query_desc, std::vector<std::vector<int>>* ids) {
    double norm_mul = norm() * query_desc.norm();
    if(norm_mul == 0) return 0;
    
    double max_score = 0.0;
    int best_shift = 0;
    for(int i = 0; i < ids->size(); i++) {
        double sum = 0.0;
        for(int j = 0; j < (*ids)[i].size(); j++) {
            sum += desc[(*ids)[i][j]]*query_desc.desc[j];
        }
        if(sum == 0) continue;
        double score = sum / norm_mul;
        if(max_score < score) {
            max_score = score;
            best_shift = i;
        }
    }
    return best_shift;
}

float ReFereeADB::getYawDiff(ReFereeA query_desc, int loop_id) {
    if(loop_id == -1) return 0;
    int idx = query_desc.maxScoreIdx(db_[loop_id], &ids_);
    float yaw_diff = 2*M_PI/query_desc.size() * (float)idx;
    if (yaw_diff > M_PI) yaw_diff = yaw_diff - 2*M_PI;
    return yaw_diff;
}

void ReFereeADB::setIDS() {
    for (int i = 0; i < desc_dim_; ++i) {
        std::vector<int> idx;
        for (int j = i; j < i + desc_dim_; ++j) {
            idx.push_back(j % desc_dim_);
        }
        ids_.push_back(idx);
    }
}