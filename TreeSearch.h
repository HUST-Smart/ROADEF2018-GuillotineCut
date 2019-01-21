#ifndef SMART_ZJL_GUILLOTINE_CUT_TREESEARCH_H
#define SMART_ZJL_GUILLOTINE_CUT_TREESEARCH_H

#include "Config.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <thread>

#include "Common.h"
#include "Utility.h"
#include "Problem.h"

namespace szx {

class TreeSearch {
    #pragma region Type
public:
    enum FlagBit {
        ROTATE = 0, // indicate item direction
        DEFECT_R = 1, // item placed in defect right
        DEFECT_U = 2, // item placed in defect upper
        BIN4 = 3, // two items placed in the same L3
        LOCKC2 = 4, // c2cpu can't move
    };

    // commmand line interface.
    struct Cli {
        static constexpr int MaxArgLen = 256;
        static constexpr int MaxArgNum = 32;

        static String InstancePathOption() { return "-p"; }
        static String SolutionPathOption() { return "-o"; }
        static String RandSeedOption() { return "-s"; }
        static String TimeoutOption() { return "-t"; }
        static String MaxIterOption() { return "-i"; }
        static String JobNumOption() { return "-j"; }
        static String RunIdOption() { return "-rid"; }
        static String EnvironmentPathOption() { return "-env"; }
        static String ConfigPathOption() { return "-cfg"; }
        static String LogPathOption() { return "-log"; }

        static String AuthorNameSwitch() { return "-name"; }
        static String HelpSwitch() { return "-h"; }

        static String AuthorName() { return "J29"; }

        // a dummy main function.
        static int run(int argc, char *argv[]);
    };

    // describe the requirements to the input and output data interface.
    struct Environment {
        static constexpr int DefaultTimeout = (1 << 30);
        static constexpr int DefaultMaxIter = (1 << 30);
        static constexpr int DefaultJobNum = 0;
        // preserved time for IO in the total given time.
        static constexpr int SaveSolutionTimeInMillisecond = 1000;

        static constexpr Duration RapidModeTimeoutThreshold = 600 * static_cast<Duration>(Timer::MillisecondsPerSecond);

        static String BatchSuffix() { return "_batch.csv"; }
        static String DefectsSuffix() { return "_defects.csv"; }
        static String SolutionSuffix() { return "_solution.csv"; }

        static String DefaultInstanceDir() { return "Instance/"; }
        static String DefaultSolutionDir() { return "Solution/"; }
        static String DefaultVisualizationDir() { return "Visualization/"; }
        static String DefaultEnvPath() { return "env.csv"; }
        static String DefaultCfgPath() { return "cfg.csv"; }
        static String DefaultLogPath() { return "log.csv"; }

        Environment(const String &instanceName, const String &solutionPath,
            int randomSeed = Random::generateSeed(), double timeoutInSecond = DefaultTimeout,
            Iteration maxIteration = DefaultMaxIter, int jobNumber = DefaultJobNum, String runId = "",
            const String &cfgFilePath = DefaultCfgPath(), const String &logFilePath = DefaultLogPath())
            : instName(instanceName), slnPath(solutionPath), randSeed(randomSeed),
            msTimeout(static_cast<Duration>(timeoutInSecond * Timer::MillisecondsPerSecond)), maxIter(maxIteration),
            jobNum(jobNumber), rid(runId), cfgPath(cfgFilePath), logPath(logFilePath), localTime(Timer::getTightLocalTime()) {}
        Environment() : Environment("", "") {}

        void load(const Map<String, char*> &optionMap);
        void load(const String &filePath);
        void loadWithoutCalibrate(const String &filePath);
        void save(const String &filePath) const;

        void calibrate(); // adjust job number and timeout to fit the platform.

        String batchPath() const { return instName + BatchSuffix(); }
        String defectsPath() const { return instName + DefectsSuffix(); }
        String solutionPath() const { return slnPath; }
        String solutionPathWithTime() const { return slnPath + "." + localTime + ".csv"; }

        String visualizPath() const { return DefaultVisualizationDir() + friendlyInstName() + "." + localTime + ".html"; }
        template<typename T>
        String visualizPath(const T &msg) const { return DefaultVisualizationDir() + friendlyInstName() + "." + localTime + "." + std::to_string(msg) + ".html"; }
        String friendlyInstName() const { // friendly to file system (without special char).
            auto pos = instName.find_last_of('/');
            return (pos == String::npos) ? instName : instName.substr(pos + 1);
        }
        String friendlyLocalTime() const { // friendly to human.
            return localTime.substr(0, 4) + "-" + localTime.substr(4, 2) + "-" + localTime.substr(6, 2)
                + "_" + localTime.substr(8, 2) + ":" + localTime.substr(10, 2) + ":" + localTime.substr(12, 2);
        }

        // essential information.
        String instName;
        String slnPath;
        int randSeed;
        Duration msTimeout;

        // optional information. highly recommended to set in benchmark.
        Iteration maxIter;
        int jobNum; // number of solvers working at the same time.
        String rid; // the id of each run.
        String cfgPath;
        String logPath;

        // auto-generated data.
        String localTime;
    };

    struct Configuration {
        TID mcin; // maximum choose item number.
        int mbpn; // maximum branch plate number.
        int mhcn; // maximum hopeful 1-cut number.
        int rcin; // repeat choose item number.
        String toBriefStr() const {
            std::ostringstream os;
            os << "GB2"
                << ";mcin=" << mcin
                << ";mbpn=" << mbpn
                << ";mhcn=" << mhcn
                << ";rcin=" << rcin;
            return os.str();
        }
        Configuration(TID MCIN = 8, int MBPN = 4, int MHCN = 1, int RCIN = 1) :mcin(MCIN), mbpn(MBPN), mhcn(MHCN), rcin(RCIN) {}
    };

    struct Rect {
        Rect() {}
        Rect(TLength width, TLength height) : w(width), h(height) {}

        TLength w; // width.
        TLength h; // height.
    };
    struct RectArea : public Rect { // a rectangular area on the plate.
        RectArea() {}
        RectArea(TCoord left, TCoord bottom, TLength width, TLength height) : Rect(width, height), x(left), y(bottom) {}

        TCoord x; // left.
        TCoord y; // bottom.
    };

    struct TreeNode {
        Depth depth; // node depth in the tree.
        TID plate; // plate id.
        TID item = Problem::InvalidItemId; // item id.
        TCoord c1cpl; // current 1-cut position left.
        TCoord c1cpr; // current 1-cut position right.
        TCoord c2cpb; // current 2-cut position bottom.
        TCoord c2cpu; // current 2-cut position up.
        TCoord c3cp; // current 3-cut position right.
        TCoord c4cp; // current 4-cut position up.
        TID cut1; // 1-cut id.
        TID cut2; // 2-cut id.
        // (flag&0x01)->(rotate);(flag&0x02)->(item place in defect side);(flag&0x04)->(item place in bin4);(flag&0x08)->(1:c2cpu change not allowed)
        Status flag = 0;

        TreeNode() {};

        TreeNode(Depth node_depth, TID plate_id, TID item_id, TCoord C1cpl, TCoord C1cpr,
            TCoord C2cpb, TCoord C2cpu, TCoord C3cp, TCoord C4cp, TID cut1_id, TID cut2_id, Status flag)
            :depth(node_depth), plate(plate_id), item(item_id), c1cpl(C1cpl), c1cpr(C1cpr),
            c2cpb(C2cpb), c2cpu(C2cpu), c3cp(C3cp), c4cp(C4cp), cut1(cut1_id), cut2(cut2_id), flag(flag) {}

        TreeNode(const TreeNode &node, const TID item_id, const int dir) :depth(node.depth + 1), plate(node.plate), item(item_id), c1cpl(node.c1cpl), c1cpr(node.c1cpr),
            c2cpb(node.c2cpb), c2cpu(node.c2cpu), c3cp(node.c3cp), c4cp(node.c4cp), cut1(node.cut1), cut2(node.cut2), flag(dir) {}
        
        void setFlagBit(const int bit_pos = FlagBit::ROTATE) { flag |= (0x0001 << bit_pos); }
        const bool getFlagBit(const int bit_pos = FlagBit::ROTATE) const { return flag & (0x0001 << bit_pos); }
    };

    #pragma endregion Type
    
    #pragma region Constructor
public:
    TreeSearch(const Problem::Input &inputData, const Environment &environment, const Configuration &config)
        : input(inputData), env(environment), cfg(config), rand(environment.randSeed),
        timer(std::chrono::milliseconds(environment.msTimeout)) {}
    #pragma endregion Constructor

    #pragma region Method
public:
    void solve();
    void record() const;
protected:
    void init();
    void greedyBranchOptimize();
    void adjustConfigure();
    int estimateOptOneCutNum(int mhcn, int rcin, List<int>& plate_1cut_num);
    Length evaluateOnePlate(const List<List<TID>>& source_batch, const List<TreeNode>& fixed_sol, const List<TreeNode>& psol, TID cur_plate);
    void getSomePlateSolutions(const TID plateId, const List<List<TID>>& source_batch, List<List<TreeNode>>& psols);
    void getOptimalPlateSolution(const TID plateId, const List<List<TID>>& source_batch, List<TreeNode>& psol);
    Area evaluateOneCut(List<List<TID>>& batch, List<TreeNode>& psol);
    void iteratorImproveWorstPlate();
    void getPlatesUsageRate(const List<TreeNode>& solution, List<double>& usage_rate);
    const int estimateDefectNumber(const TreeNode& resume_point, const List<List<TID>>& source_bacth);
    const int createItemBatchs(int nums, const TreeNode& resume_point, const List<List<TID>>& source_batch, List<List<List<TID>>>& target_batch);
    double optimizeOneCut(const TreeNode &resume_point, List<List<TID>> &batch, List<TreeNode> &solution);
    double optimizePlateTail(const TreeNode &resume_point, List<List<TID>> &batch, List<TreeNode> &solution);
    void optimizeTotalProblem();
    bool partialBranch(const TreeNode &old, const List<List<TID>> &batch, const List<TreeNode> &cur_parsol, List<TreeNode> &branch_nodes);
    void totalBranch(const TreeNode &old, const List<List<TID>> &batch, const List<TreeNode> &cur_parsol, List<TreeNode> &branch_nodes);
    const bool constraintCheck(const TreeNode &old, const List<TreeNode> &cur_parsol, TreeNode &node);
    const TCoord sliptoDefectRight(const RectArea &area, const TID plate) const;
    const TCoord sliptoDefectUp(const RectArea &area, const TID plate) const;
    const bool defectConflictArea(const RectArea &area, const TID plate) const;
    const Length getLowBound(const TreeNode &cur_node, Area left_item_area) const;
    void toOutput(List<TreeNode> &sol);
    const TCoord getC1cpr(const List<TreeNode> &sol, const int index, const TID cur_plate, const TID cur_cut1) const;
    const TCoord getC2cpu(const List<TreeNode> &sol, const int index, const TID cur_cut1, const TID cur_cut2) const;
    bool check(Length &checkerObj) const;
    const double getScrapWasteRate(List<TreeNode>& sol) const;
    #pragma endregion Method

    #pragma region Field
public:
    Problem::Input input;
    Problem::Output output;

    struct {
        List<Rect> items;
        List<RectArea> defects;
        List<Area> item_area; // item area size of every item.
        List<TID> item2stack; //defect identity to stack identity

        List<List<TID>> stacks; // stacks[s][i] is the itemId of the i_th item in the stack s.
        List<List<TID>> plates_x; // plates[p][i] is the defectId of the i_th defect on plate p, sorted by defect x position.
        List<List<TID>> plates_y; // plates[p][i] is the defectId of the i_th defect on plate p, sorted by defect y position.
    } aux;

    struct {
        ZeroBasedConsecutiveIdMap<TID, TID, Problem::MaxItemNum> item;
        ZeroBasedConsecutiveIdMap<TID, TID, Problem::MaxStackNum> stack;
        ZeroBasedConsecutiveIdMap<TID, TID, Problem::MaxDefectNum> defect;
        ZeroBasedConsecutiveIdMap<TID, TID, Problem::MaxPlateNum> plate;
    } idMap;

    struct {
        double scrap_rate = 0.0;
        int generation_stamp = 0;
        String toStr() const {
            std::ostringstream os;
            os << "sr=" << scrap_rate*100 << "%"
                << ";gs=" << generation_stamp;
            return os.str();
        }
    } info;

    Configuration cfg;
    Environment env;
    List<TreeNode> best_solution;
    Length best_objective = input.param.plateNum*input.param.plateWidth;
    double best_usage_rate = 1.0;
    Area total_item_area = 0;

    Random rand; // all random number in TSolver must be generated by this.
    Timer timer; // the solve() should return before it is timeout.
    int generation = 0; // the constructed complete solution number.
    int support_thread = 1;

    #pragma endregion Field
};

}

#endif // SMART_ZJL_GUILLOTINE_CUT_TREESEARCH_H
