////////////////////////////////
/// usage : 1.	read csv file into an jagged array.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GUILLOTINE_CUT_CSV_READER_H
#define SMART_SZX_GUILLOTINE_CUT_CSV_READER_H


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <vector>


// [off] increase readability instead of stabilization in debug mode (avoid stack overflow).
#define SMART_SZX_GUILLOTINE_CUT_CSV_READER_RECURSIVE_VERSION  0


namespace szx {

class CsvReader {
public:
    using Row = std::vector<char*>;


    static constexpr char CommaChar = ';';


    const std::vector<Row>& scan(std::ostringstream &oss) {
        oss << '\n'; // add a sentinel so that end of file check is only necessary in onNewLine().
        data = oss.str();
        begin = const_cast<char*>(data.data());  // the const cast can be omitted since C++17.
        end = begin + data.size();

        #if SMART_SZX_GUILLOTINE_CUT_CSV_READER_RECURSIVE_VERSION
        onNewLine(begin);
        #else
        onNewLine_opt(begin);
        #endif // SMART_SZX_GUILLOTINE_CUT_CSV_READER_RECURSIVE_VERSION

        return rows;
    }
    const std::vector<Row>& scan(std::ifstream &ifs) {
        std::ostringstream oss;
        oss << ifs.rdbuf();
        return scan(oss);
    }

protected:
    #if SMART_SZX_GUILLOTINE_CUT_CSV_READER_RECURSIVE_VERSION
    void onNewLine(char *s) {
        while ((s != end) && (NewLineChars.find(*s) != NewLineChars.end())) { ++s; } // remove empty lines.
        if (s == end) { return; }

        rows.push_back(Row());

        onSpace(s);
    }

    void onSpace(char *s) {
        while (SpaceChars.find(*s) != SpaceChars.end()) { ++s; } // trim spaces.

        onValue(s);
    }

    void onValue(char *s) {
        rows.back().push_back(s);

        char c = *s;
        if (EndCellChars.find(c) == EndCellChars.end()) {
            while (EndCellChars.find(*(++s)) == EndCellChars.end()) {}
            c = *s;

            char *space = s;
            while (SpaceChars.find(*(space - 1)) != SpaceChars.end()) { --space; }
            *space = 0; // trim spaces and remove comma or line ending.
        } else { // empty cell.
            *s = 0;
        }

        ++s;
        (NewLineChars.find(c) != NewLineChars.end()) ? onNewLine(s) : onSpace(s);
    }
    #else // in case there is no Tail-Call Optimization which leads to the stack overflow.
    void onNewLine_opt(char *s) {
Label_OnNewLine:
        while ((s != end) && (NewLineChars.find(*s) != NewLineChars.end())) { ++s; } // remove empty lines.
        if (s == end) { return; }

        rows.push_back(Row());

Label_OnSpace:
        while (SpaceChars.find(*s) != SpaceChars.end()) { ++s; } // trim spaces.

//Label_OnValue:
        rows.back().push_back(s);

        char c = *s;
        if (EndCellChars.find(c) == EndCellChars.end()) {
            while (EndCellChars.find(*(++s)) == EndCellChars.end()) {}
            c = *s;

            char *space = s;
            while (SpaceChars.find(*(space - 1)) != SpaceChars.end()) { --space; }
            *space = 0; // trim spaces and remove comma or line ending.
        } else { // empty cell.
            *s = 0;
        }

        ++s;
        if (NewLineChars.find(c) != NewLineChars.end()) {
            goto Label_OnNewLine;
        } else {
            goto Label_OnSpace;
        }
    }
    #endif // SMART_SZX_GUILLOTINE_CUT_CSV_READER_RECURSIVE_VERSION

    // TODO[szx][2]: handle quote (comma will not end cell).
    // EXTEND[szx][5]: make trim space configurable.

    static const std::set<char> NewLineChars;
    static const std::set<char> SpaceChars;
    static const std::set<char> EndCellChars;

    char *begin;
    const char *end;

    std::string data;
    std::vector<Row> rows;
};

}


#endif // SMART_SZX_GUILLOTINE_CUT_CSV_READER_H
