#include "ltslam/RosParamServer.h"

#include <fstream>
#include <sstream>
#include <map>
#include <cstdlib>

static std::string g_config_path;
void setLtslamConfigPath(const std::string& path) { g_config_path = path; }

namespace {
// trivial flat-YAML reader: handles "  key: value" (1-level nesting under
// "ltslam:"), strips quotes and trailing # comments. Sufficient for the
// ltslam params file.
std::map<std::string, std::string> readFlatYaml(const std::string& path)
{
    std::map<std::string, std::string> kv;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        // strip comments
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        // trim helper
        auto trim = [](std::string s) {
            size_t a = s.find_first_not_of(" \t\r\n");
            size_t b = s.find_last_not_of(" \t\r\n");
            return (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
        };
        key = trim(key);
        val = trim(val);
        if (key.empty() || val.empty()) continue;   // section headers like "ltslam"
        // strip surrounding quotes
        if (val.size() >= 2 && (val.front() == '"' || val.front() == '\'') && val.back() == val.front())
            val = val.substr(1, val.size() - 2);
        kv[key] = val;
    }
    return kv;
}
} // namespace

RosParamServer::RosParamServer()
{
    auto kv = readFlatYaml(g_config_path);
    auto get = [&](const std::string& k, const std::string& def) {
        auto it = kv.find(k);
        return it == kv.end() ? def : it->second;
    };

    sessions_dir_      = get("sessions_dir", "/");
    central_sess_name_ = get("central_sess_name", "01");
    query_sess_name_   = get("query_sess_name", "02");
    save_directory_    = get("save_directory", "/LTslam/");

    int unused = system((std::string("exec rm -r ") + save_directory_ + " 2>/dev/null").c_str());
    (void)unused;
    unused = system((std::string("mkdir -p ") + save_directory_).c_str());

    is_display_debug_msgs_     = (get("is_display_debug_msgs", "false") == "true");
    numberOfCores              = std::stoi(get("numberOfCores", "4"));
    kNumSCLoopsUpperBound      = std::stoi(get("kNumSCLoopsUpperBound", "10"));
    kNumRSLoopsUpperBound      = std::stoi(get("kNumRSLoopsUpperBound", "10"));
    loopFitnessScoreThreshold  = std::stof(get("loopFitnessScoreThreshold", "200.0"));
    loopMaxTranslation         = std::stof(get("loopMaxTranslation", "3.0"));

    std::cout << "[ltslam config] sessions_dir=" << sessions_dir_
              << " central=" << central_sess_name_ << " query=" << query_sess_name_
              << " save=" << save_directory_ << std::endl;
}
