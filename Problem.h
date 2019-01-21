////////////////////////////////
/// usage : 1.	data that identifies a guillotine cut problem and its solution.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GUILLOTINE_CUT_PROBLEM_H
#define SMART_SZX_GUILLOTINE_CUT_PROBLEM_H


#include "Config.h"

#include "Common.h"


namespace szx {

class Problem {
    #pragma region Type
public:
    struct Input {
        struct Item {
            ID id;
            Length width;
            Length height;
            ID stack;
            ID seq;
        };

        struct Defect {
            ID id;
            ID plateId;
            Coord x;
            Coord y;
            Length width;
            Length height;
        };

        struct Param {
            ID plateNum;
            Length plateWidth;
            Length plateHeight;
            Length minL1Width;
            Length maxL1Width;
            Length minL2Height;
            Length minWasteWidth;
            Length minWasteHeight;
        };

        enum BatchColumn { ItemId, ItemWidth, ItemHeight, Stack, Seq };
        enum DefectsColumn { DefectId, PlateId, DefectX, DefectY, DefectWidth, DefectHeight };

        void loadParam() {
            // EXTEND[szx][9]: load from file when provided?
            param.plateNum = 100;
            param.plateWidth = 6000;
            param.plateHeight = 3210;
            param.minL1Width = 100;
            param.maxL1Width = 3500;
            param.minL2Height = 100;
            param.minWasteWidth = 20;
            param.minWasteHeight = 20;
        }
        bool loadBatch(const String &batchFilePath);
        bool loadDefects(const String &defectsFilePath);
        bool load(const String &batchFilePath, const String &defectsFilePath) {
            loadParam();
            return (loadBatch(batchFilePath) && loadDefects(defectsFilePath));
        }

        List<Item> batch;
        List<Defect> defects;
        Param param;
    };

    struct Output {
        struct Node {
            using Type = int;

            enum SpecialType { Residual = -3, Branch = -2, Waste = -1, ItemBegin = 0 };

            ID plateId;
            ID id;
            Coord x;
            Coord y;
            Length width;
            Length height;
            Type type; // Item.id or Type.
            int cut; // 0 for plate. 1, 2, 3, 4 for real cuts.
            ID parent;
            Node() {}
            Node(ID plateID, ID nodeID, Coord pos_x, Coord pos_y, Length w, Length h, Type node_type, int cut_depth, ID parentID) :
                plateId(plateID), id(nodeID), x(pos_x), y(pos_y), width(w), height(h), type(node_type), cut(cut_depth), parent(parentID) {}
        };

        enum SlnColumn { PlateId, NodeId, NodeX, NodeY, NodeWidth, NodeHeight, NodeType, CutNum, Parent };

        static constexpr Length MaxWidth = 6000 * 100;

        Output() : totalWidth(MaxWidth) {}
        void load(const String &filePath);
        void save(const String &filePath) const;

        Length totalWidth; // objective value.
        List<Node> nodes; // solution vector.
    };
    #pragma endregion Type

    #pragma region Constant
public:
    static constexpr ID MaxItemNum = 700;
    static constexpr ID MaxStackNum = MaxItemNum;
    static constexpr ID MaxPlateNum = 100;
    static constexpr ID MaxDefectNumPerPlate = 8;
    static constexpr ID MaxDefectNum = MaxPlateNum * MaxDefectNumPerPlate;

    static constexpr ID InvalidItemId = -1;
    #pragma endregion Constant

    #pragma region Constructor
public:
    #pragma endregion Constructor

    #pragma region Method
public:
    #pragma endregion Method

    #pragma region Field
public:
    #pragma endregion Field
}; // Problem

}


#endif // SMART_SZX_GUILLOTINE_CUT_PROBLEM_H
