#include "Problem.h"

#include <cstdlib>

#include "Utility.h"
#include "LogSwitch.h"
#include "CsvReader.h"


using namespace std;


namespace szx {

#pragma region Problem::Input
bool Problem::Input::loadBatch(const String &batchFilePath) {
    ifstream ifs(batchFilePath);
    if (!ifs.is_open()) {
        Log(Log::Fatal) << "fail to open batch file." << endl;
        return false;
    }

    CsvReader cr;
    const vector<CsvReader::Row> &rows(cr.scan(ifs));
    batch.reserve(rows.size() - 2);
    for (auto r = rows.begin() + 1; r != rows.end(); ++r) { // skip header.
        batch.push_back({
            atoi(r->at(BatchColumn::ItemId)),
            atoi(r->at(BatchColumn::ItemWidth)),
            atoi(r->at(BatchColumn::ItemHeight)),
            atoi(r->at(BatchColumn::Stack)),
            atoi(r->at(BatchColumn::Seq))
        });
    }

    return true;
}

bool Problem::Input::loadDefects(const String &defectsFilePath) {
    ifstream ifs(defectsFilePath);
    if (!ifs.is_open()) {
        Log(Log::Fatal) << "fail to open batch file." << endl;
        return false;
    }

    CsvReader cr;
    const vector<CsvReader::Row> &rows(cr.scan(ifs));
    defects.reserve(rows.size() - 2);
    for (auto r = rows.begin() + 1; r != rows.end(); ++r) { // skip header.
        defects.push_back({
            atoi(r->at(DefectsColumn::DefectId)),
            atoi(r->at(DefectsColumn::PlateId)),
            atoi(r->at(DefectsColumn::DefectX)),
            atoi(r->at(DefectsColumn::DefectY)),
            atoi(r->at(DefectsColumn::DefectWidth)),
            atoi(r->at(DefectsColumn::DefectHeight))
        });
    }

    return true;
}
#pragma endregion Problem::Input

#pragma region Problem::Output
void Problem::Output::load(const String &filePath) {
    ifstream ifs(filePath);
    if (!ifs.is_open()) {
        Log(Log::Error) << "fail to open solution file." << endl;
        return;
    }

    CsvReader cr;
    const vector<CsvReader::Row> &rows(cr.scan(ifs));
    nodes.reserve(rows.size() - 2);
    for (auto r = rows.begin() + 1; r != rows.end(); ++r) { // skip header.
        nodes.push_back({
            atoi(r->at(SlnColumn::PlateId)),
            atoi(r->at(SlnColumn::NodeId)),
            atoi(r->at(SlnColumn::NodeX)),
            atoi(r->at(SlnColumn::NodeY)),
            atoi(r->at(SlnColumn::NodeWidth)),
            atoi(r->at(SlnColumn::NodeHeight)),
            atoi(r->at(SlnColumn::NodeType)),
            atoi(r->at(SlnColumn::CutNum)),
            atoi(r->at(SlnColumn::Parent))
        });
    }
}

void Problem::Output::save(const String &filePath) const {
    ofstream ofs(filePath);

    ofs << "PLATE_ID;NODE_ID;X;Y;WIDTH;HEIGHT;TYPE;CUT;PARENT" << endl;
    for (auto n = nodes.begin(); n != nodes.end(); ++n) {
        ofs << n->plateId << CsvReader::CommaChar
            << n->id << CsvReader::CommaChar
            << n->x << CsvReader::CommaChar
            << n->y << CsvReader::CommaChar
            << n->width << CsvReader::CommaChar
            << n->height << CsvReader::CommaChar
            << n->type << CsvReader::CommaChar
            << n->cut << CsvReader::CommaChar;
        if (n->parent >= 0) { ofs << n->parent; }
        ofs << endl;
    }
}
#pragma endregion Problem::Output

}
